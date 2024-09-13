//! Configuration module
//!
//! Provides a centralized configuration store for the device.

use binrw::{binrw, io::Cursor, BinRead, BinResult, BinWrite};
use embassy_embedded_hal::adapter::{BlockingAsync, YieldingAsync};
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use embedded_storage::{ReadStorage, Storage};
use esp_storage::{FlashStorage, FlashStorageError};
use sequential_storage::cache::KeyPointerCache;
use sequential_storage::map::store_item;
use sequential_storage::{
    self,
    map::{fetch_item, Value},
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[binrw]
#[brw(little)]
pub struct UwbConfig {
    /// 16-bit short address
    pub short_address: u16,
    /// UWB MAC address
    pub long_address: [u8; 6],
    /// PAN ID
    pub pan_id: u16,
    /// Channel
    pub channel: u8,
}

impl Default for UwbConfig {
    fn default() -> Self {
        Self {
            short_address: 0,
            long_address: [0; 6],
            pan_id: 0,
            channel: 0,
        }
    }
}

pub type KeyType = [u8; 8]; // 64-bit key
const PAGE_COUNT: usize = 16;

pub struct ConfigurationStore {
    storage_partition: esp_partition_table::PartitionEntry,
    cache: KeyPointerCache<PAGE_COUNT, KeyType, 16>,
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
        })
    }

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
    pub async fn get<'b, T: Value<'b>>(
        &mut self,
        buffer: &'b mut [u8],
        key: &KeyType,
    ) -> Result<Option<T>, sequential_storage::Error<FlashStorageError>> {
        let mut storage = YieldingAsync::new(BlockingAsync::new(FlashStorage::new()));
        fetch_item(
            &mut storage,
            self.flash_range(),
            &mut self.cache,
            buffer,
            key,
        )
        .await
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
    pub async fn set<'a, T: Value<'a>>(
        &mut self,
        key: &KeyType,
        value: T,
    ) -> Result<(), sequential_storage::Error<FlashStorageError>> {
        let mut storage = YieldingAsync::new(BlockingAsync::new(FlashStorage::new()));
        let mut buffer = [0; 256];
        store_item(
            &mut storage,
            self.flash_range(),
            &mut self.cache,
            &mut buffer,
            key,
            &value,
        )
        .await
    }
}
