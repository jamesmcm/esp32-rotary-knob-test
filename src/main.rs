use std::sync::mpsc::{Receiver, Sender};
use std::sync::Mutex;

use anyhow::{anyhow, bail, Result};
use embedded_hal::digital::v2::InputPin;
use esp_idf_hal::gpio::{Pin, PinDriver, Pull};
use esp_idf_hal::prelude::Peripherals;
use esp_idf_svc::eventloop::{Background, EspEventLoop, User};
use esp_idf_svc::systime;
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use esp_idf_sys::{
    esp, nvs_flash_deinit, nvs_flash_erase, nvs_flash_init, ESP_ERR_NVS_NEW_VERSION_FOUND,
    ESP_ERR_NVS_NO_FREE_PAGES,
};
use log::{info, trace};
use once_cell::sync::OnceCell;
use rotary_encoder_hal::{Direction, Rotary};

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum RotaryKnobEvent {
    TurnedClockwise,
    TurnedCounterClockwise,
    ButtonPressed,
    ButtonReleased,
}

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum PinId {
    Button,
    A,
    B,
}

mod event {
    use super::PinId;
    use esp_idf_svc::eventloop::{
        EspEventFetchData, EspEventPostData, EspTypedEventDeserializer, EspTypedEventSerializer,
        EspTypedEventSource,
    };

    #[derive(Copy, Clone, Debug)]
    pub struct EventLoopMessage(pub PinId);

    impl EventLoopMessage {
        pub fn new(data: PinId) -> Self {
            Self(data)
        }
    }

    impl EspTypedEventSource for EventLoopMessage {
        fn source() -> *const core::ffi::c_char {
            b"DEMO-SERVICE\0".as_ptr() as *const _
        }
    }

    impl EspTypedEventSerializer<EventLoopMessage> for EventLoopMessage {
        fn serialize<R>(
            event: &EventLoopMessage,
            f: impl for<'a> FnOnce(&'a EspEventPostData) -> R,
        ) -> R {
            f(&unsafe { EspEventPostData::new(Self::source(), Self::event_id(), event) })
        }
    }

    impl EspTypedEventDeserializer<EventLoopMessage> for EventLoopMessage {
        fn deserialize<R>(
            data: &EspEventFetchData,
            f: &mut impl for<'a> FnMut(&'a EventLoopMessage) -> R,
        ) -> R {
            f(unsafe { data.as_payload() })
        }
    }
}

fn event_loop() -> &'static Mutex<EspEventLoop<User<Background>>> {
    static INSTANCE: OnceCell<Mutex<EspEventLoop<User<Background>>>> = OnceCell::new();
    INSTANCE.get_or_init(|| {
        #[allow(unused)]
        let eventloop = EspEventLoop::<User<Background>>::new(&Default::default()).unwrap();
        Mutex::new(eventloop)
    })
}

fn handle_interrupt(pin_id: PinId) {
    event_loop()
        .lock()
        .unwrap()
        .post(&event::EventLoopMessage::new(pin_id), None)
        .unwrap();
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    let result = unsafe { esp!(nvs_flash_init()) };

    if let Err(e) = result {
        match e.code() {
            ESP_ERR_NVS_NO_FREE_PAGES | ESP_ERR_NVS_NEW_VERSION_FOUND => unsafe {
                esp!(nvs_flash_erase()).unwrap();
                esp!(nvs_flash_init()).unwrap();
            },
            _ => Err(anyhow!(e)).unwrap(),
        };
    }

    let peripherals = Peripherals::take().unwrap();
    let sys_loop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();

    let mut io2 = PinDriver::input(peripherals.pins.gpio2).unwrap();
    let mut io32 = PinDriver::input(peripherals.pins.gpio32).unwrap();
    let mut io33 = PinDriver::input(peripherals.pins.gpio33).unwrap();

    io2.set_pull(Pull::Up).unwrap();
    io32.set_pull(Pull::Up).unwrap();
    io33.set_pull(Pull::Up).unwrap();

    io2.set_interrupt_type(esp_idf_hal::gpio::InterruptType::AnyEdge)
        .unwrap();
    io32.set_interrupt_type(esp_idf_hal::gpio::InterruptType::AnyEdge)
        .unwrap();
    io33.set_interrupt_type(esp_idf_hal::gpio::InterruptType::AnyEdge)
        .unwrap();

    unsafe {
        io2.subscribe(|| handle_interrupt(PinId::Button)).unwrap();
        io32.subscribe(|| handle_interrupt(PinId::B)).unwrap();
        io33.subscribe(|| handle_interrupt(PinId::A)).unwrap();
    }
    // Moved into closure
    let mut enc = Rotary::new(io33, io32);
    let mut button_pressed = false;
    let systime = esp_idf_svc::systime::EspSystemTime;
    let mut prev_time = systime.now();
    // TODO try async?
    let (tx, rx): (Sender<RotaryKnobEvent>, Receiver<RotaryKnobEvent>) = std::sync::mpsc::channel();

    let sub = event_loop()
        .lock()
        .unwrap()
        .subscribe(move |e: &event::EventLoopMessage| {
            trace!("Received event: {:?}", e);
            let new_time = systime.now();
            if new_time - prev_time < core::time::Duration::from_millis(100) {
                trace!("Discarding event due to rapid timing: {:?}", e);
                enc.update().unwrap();
                return;
            }
            prev_time = new_time;
            match e.0 {
                PinId::A | PinId::B => {
                    let maybe_event = match enc.update().unwrap() {
                        Direction::Clockwise => Some(RotaryKnobEvent::TurnedClockwise),
                        Direction::CounterClockwise => {
                            Some(RotaryKnobEvent::TurnedCounterClockwise)
                        }
                        Direction::None => None,
                    };
                    if let Some(event) = maybe_event {
                        tx.send(event).unwrap();
                    }
                }
                PinId::Button => {
                    let c = io2.is_low();
                    if button_pressed != c {
                        button_pressed = c;
                        if button_pressed {
                            tx.send(RotaryKnobEvent::ButtonPressed).unwrap();
                        } else {
                            tx.send(RotaryKnobEvent::ButtonReleased).unwrap();
                        }
                    }
                }
            }
        })
        .unwrap();

    info!("Entering loop...");
    loop {
        std::thread::sleep(std::time::Duration::from_millis(50));
        info!("Received RotaryKnob event: {:?}", rx.recv().unwrap());
    }
}
