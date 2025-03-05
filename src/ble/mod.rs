use std::sync::Mutex;

use oneshot::{Receiver, Sender};

pub use central::connect_client;
pub use peripheral::serve_peripheral;

mod central;
mod peripheral;

struct RemovableSender<T>(Mutex<Option<Sender<T>>>);

impl<T> RemovableSender<T> {
    fn new() -> (RemovableSender<T>, Receiver<T>) {
        let (tx, rx) = oneshot::channel();
        (Self(Mutex::new(Some(tx))), rx)
    }

    fn send(&self, value: T) -> eyre::Result<()> {
        let Some(sender) = self.0.lock().unwrap().take() else {
            eyre::bail!("sender was already taken");
        };

        sender
            .send(value)
            .map_err(|_| eyre::eyre!("failed to send"))
    }
}
