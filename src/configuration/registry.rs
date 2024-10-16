//! Configuration registry
//!
//! This file stores the "schema" of the configuration.
//! It is used to validate the configuration and to provide a human-readable interface for the user to
//! interact with the configuration.

use core::any::TypeId;
use heapless::FnvIndexMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Entry {
    pub type_id: TypeId,
    pub name: &'static str,
}

pub struct Registry {
    entries: FnvIndexMap<super::KeyType, Entry, 128>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Error {
    KeyAlreadyExists,
    RegistryFull,
}

impl Registry {
    pub fn new() -> Self {
        Self {
            entries: FnvIndexMap::new(),
        }
    }

    pub fn register<T: 'static>(&mut self, key: &super::KeyType) -> Result<(), Error> {
        if self.entries.contains_key(key) {
            return Err(Error::KeyAlreadyExists);
        }
        self.entries
            .insert(
                *key,
                Entry {
                    type_id: TypeId::of::<T>(),
                    name: core::any::type_name::<T>(),
                },
            )
            .map_err(|_| Error::RegistryFull)?;
        Ok(())
    }

    pub fn get(&self, key: &super::KeyType) -> Option<&Entry> {
        self.entries.get(key)
    }

    pub fn iter(&self) -> impl Iterator<Item = (&super::KeyType, &Entry)> {
        self.entries.iter()
    }
}
