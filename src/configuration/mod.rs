//! Configuration module
//!
//! Provides a centralized configuration store for the device.

use core::any::TypeId;

use embassy_embedded_hal::adapter::{BlockingAsync, YieldingAsync};
use esp_hal::macros::ram;
use esp_storage::{FlashStorage, FlashStorageError};
use sequential_storage::cache::KeyPointerCache;
use sequential_storage::map::store_item;
use sequential_storage::{
    self,
    map::{fetch_item, Value},
};

pub mod registry;

pub use registry::Registry;

pub type KeyType = [u8; 8]; // 64-bit key
const PAGE_COUNT: usize = 16;

#[derive(Debug)]
pub enum ConfigurationError {
    KeyTypeMismatch,
    #[allow(dead_code)]
    StorageError(sequential_storage::Error<FlashStorageError>),
}

pub struct ConfigurationStore {
    storage_partition: esp_partition_table::PartitionEntry,
    cache: KeyPointerCache<PAGE_COUNT, KeyType, 16>,
    pub registry: Registry,
}

impl ConfigurationStore {
    pub fn new() -> Option<Self> {
        // --- Configuration ---
        let partition_table = esp_partition_table::PartitionTable::default();
        let mut flash_storage = FlashStorage::new();
        let mut config_part: Option<esp_partition_table::PartitionEntry> = None;
        for partition in partition_table.iter_storage(&mut flash_storage, false) {
            let partition = partition.unwrap();
            defmt::debug!(
                "Partition: {:?}, offset: {:#x}",
                partition.name(),
                partition.offset
            );
            if partition.name() == "config" {
                config_part = Some(partition);
            }
        }
        if config_part.is_none() {
            defmt::error!("No storage partition found");
            return None;
        }

        // --- Load configuration ---
        let config_part = config_part.unwrap();

        // Check page count
        let page_count = config_part.size as u32 / esp_storage::FlashStorage::SECTOR_SIZE;
        if page_count > PAGE_COUNT as u32 {
            defmt::error!("Page count too high: {}", page_count);
            return None;
        }

        Some(Self {
            storage_partition: config_part,
            cache: KeyPointerCache::new(),
            registry: Registry::new(),
        })
    }

    /// Get the flash range for the configuration store
    #[ram]
    fn flash_range(&self) -> core::ops::Range<u32> {
        self.storage_partition.offset
            ..self.storage_partition.offset + self.storage_partition.size as u32
    }

    /// Get a value from the configuration store
    ///
    /// # Arguments
    /// * `buffer` - A buffer for the read operation, must be large enough to hold the value
    /// * `key` - The key to read
    ///
    /// # Returns
    /// * `Ok(Some(T))` - The value was found and returned
    /// * `Ok(None)` - The value was not found
    /// * `Err(sequential_storage::Error<FlashStorageError>)` - An error occurred during the read operation
    #[ram]
    pub async fn get<'b, T: Value<'b> + 'static>(
        &mut self,
        buffer: &'b mut [u8],
        key: &KeyType,
    ) -> Result<Option<T>, ConfigurationError> {
        let type_id = self.registry.get(key);
        if type_id.is_some() && type_id.unwrap().type_id != TypeId::of::<T>() {
            return Err(ConfigurationError::KeyTypeMismatch);
        }

        let mut storage = YieldingAsync::new(BlockingAsync::new(FlashStorage::new()));
        fetch_item::<_, T, _>(
            &mut storage,
            self.flash_range(),
            &mut self.cache,
            buffer,
            key,
        )
        .await
        .map_err(ConfigurationError::StorageError)
    }

    /// Set a value in the configuration store
    ///
    /// # Arguments
    /// * `key` - The key to set
    /// * `value` - The value to set
    ///
    /// # Returns
    /// * `Ok(())` - The value was set
    /// * `Err(sequential_storage::Error<FlashStorageError>)` - An error occurred during the write operation
    #[ram]
    pub async fn set<'a, T: Value<'a> + 'static>(
        &mut self,
        key: &KeyType,
        value: T,
    ) -> Result<(), ConfigurationError> {
        let type_id = TypeId::of::<T>();
        let key_type = self.registry.get(key);

        if key_type.is_some() && key_type.unwrap().type_id != type_id {
            return Err(ConfigurationError::KeyTypeMismatch);
        }

        let mut storage = YieldingAsync::new(BlockingAsync::new(FlashStorage::new()));
        let mut buffer = [0; 256];
        store_item::<_, T, _>(
            &mut storage,
            self.flash_range(),
            &mut self.cache,
            &mut buffer,
            key,
            &value,
        )
        .await
        .map_err(ConfigurationError::StorageError)
    }
}
