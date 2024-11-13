//! This module is responsible for transferring touch events to the UI framework

use defmt::{debug, error, info, warn};
use embassy_nrf::twim::{self, Error};
use embassy_time::{Duration, Instant};
use slint::platform::{software_renderer::MinimalSoftwareWindow, PointerEventButton, WindowEvent};

use crate::{
    cst816s::{Cst816s, TouchData},
    ui_task::extensions::PrintableWindowEvent,
};

pub(super) struct TouchHandler {
    window: alloc::rc::Rc<MinimalSoftwareWindow>,
    touch_notified_to_ui: bool,
    last_touch_instant: Option<Instant>,
    last_touch_data: Option<TouchData>,
    first_touch_event: Option<Instant>,
}

impl TouchHandler {
    pub(super) fn new(window: alloc::rc::Rc<MinimalSoftwareWindow>) -> Self {
        Self {
            window,
            touch_notified_to_ui: false,
            last_touch_instant: None,
            last_touch_data: None,
            first_touch_event: None,
        }
    }

    pub(super) fn handle_new_touch_data(&mut self, touch_data: Result<TouchData, Error>) {
        match touch_data {
            Ok(touch_data) => {
                let event: Option<WindowEvent>;
                (self.touch_notified_to_ui, event) = Self::evaluate_touch_data(
                    touch_data,
                    self.last_touch_data,
                    self.window.scale_factor(),
                    self.touch_notified_to_ui,
                );
                self.last_touch_data = Some(touch_data);
                if touch_data.is_touching {
                    if let Some(WindowEvent::PointerPressed {
                        position: _,
                        button: _,
                    }) = event
                    {
                        self.first_touch_event = Some(Instant::now());
                    }
                    if let Some(WindowEvent::PointerMoved { position: _ }) = event {
                        if let Some(first_touch_event) = self.first_touch_event {
                            let i = Instant::now()
                                .checked_duration_since(first_touch_event)
                                .unwrap();
                            info!("Delta T {}ms", i.as_millis());
                        }
                        self.first_touch_event = None;
                    }
                    self.last_touch_instant = Some(Instant::now());
                } else {
                    self.last_touch_instant = None;
                }
                if let Some(event) = event {
                    debug!("Event {}", PrintableWindowEvent(&event));
                    self.window.dispatch_event(event);
                }
            }
            Err(e) => error!("Touch read error => {}", e),
        }
    }

    /// Every now and then, depending on the UI load, we happen to miss touch events
    /// In general it's not a big deal, an issue occurs though when the last "released" event is not
    /// propagated to the window manager, the UI behave not as expected anymore
    /// Hence this function must be called periodically after the UI event loop has completed.
    /// Alternatively, we would have to change the touch-controller to issue the I2C read in interrupt context and
    /// such that the `async` `wait_event` function will directly receive the new data whenever available.
    /// Otherwise, it's likely that we're always going to miss data.
    pub(super) fn check_missed_event<TWIM: twim::Instance>(
        &mut self,
        touch_controller: &mut Cst816s<'_, TWIM>, // Todo add a touch-read-trait and use it here as impl Trait
    ) {
        if let Some(touch_instant) = self.last_touch_instant {
            // Check if the touch wasn't released for long, it seems like we miss interrupts.
            // In this way we can be sure that we properly detect release actions.
            if Instant::now()
                .checked_duration_since(touch_instant)
                .unwrap_or(Duration::from_secs(0))
                > Duration::from_millis(100)
            {
                if let Ok(touch_data) = touch_controller.read_touch_data() {
                    let event: Option<WindowEvent>;
                    (self.touch_notified_to_ui, event) = Self::evaluate_touch_data(
                        touch_data,
                        self.last_touch_data,
                        self.window.scale_factor(),
                        self.touch_notified_to_ui,
                    );
                    match event {
                        Some(WindowEvent::PointerReleased {
                            position: _,
                            button: _,
                        }) => {
                            warn!("It appears we did really miss the release event");
                            self.window
                                .dispatch_event(event.expect("Safe thanks to the prev. checks"));
                        }
                        _ => {
                            debug!("It appears that we didn't miss the release event after all...");
                        }
                    }
                    self.last_touch_instant = None;
                }
            }
        }
    }

    fn evaluate_touch_data(
        touch_data: TouchData,
        last_touch_data: Option<TouchData>,
        scale_factor: f32,
        mut touch_notified_to_ui: bool,
    ) -> (bool, Option<WindowEvent>) {
        debug!("Touch {:?}", touch_data);

        let touch_position = slint::PhysicalPosition::new(touch_data.x as _, touch_data.y as _)
            .to_logical(scale_factor);
        let event = if touch_data.is_touching {
            if !touch_notified_to_ui {
                touch_notified_to_ui = true;
                Some(slint::platform::WindowEvent::PointerPressed {
                    position: touch_position,
                    button: PointerEventButton::Left,
                })
            } else {
                // Analyze touch input only in case of a change
                if last_touch_data.is_some_and(|t| {
                    touch_data.x != t.x
                        || touch_data.y != t.y
                        || touch_data.is_touching != t.is_touching
                }) {
                    Some(slint::platform::WindowEvent::PointerMoved {
                        position: touch_position,
                    })
                } else {
                    None
                }
            }
        } else {
            touch_notified_to_ui = false;
            Some(slint::platform::WindowEvent::PointerReleased {
                position: touch_position,
                button: PointerEventButton::Left,
            })
        };
        (touch_notified_to_ui, event)
    }
}
