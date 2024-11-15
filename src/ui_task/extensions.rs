use defmt::{Format, Formatter};
use slint::platform::WindowEvent;

pub(super) struct PrintableWindowEvent<'a>(pub(super) &'a WindowEvent);
impl<'a> Format for PrintableWindowEvent<'a> {
    fn format(&self, fmt: Formatter) {
        match &self.0 {
            WindowEvent::PointerPressed {
                position,
                button: _,
            } => {
                defmt::write!(fmt, "PointerPressed x={}, y={}", position.x, position.y)
            }
            WindowEvent::PointerReleased {
                position,
                button: _,
            } => {
                defmt::write!(fmt, "PointerReleased x={}, y={}", position.x, position.y)
            }
            WindowEvent::PointerMoved { position } => {
                defmt::write!(fmt, "PointerMoved x={}, y={}", position.x, position.y)
            }
            WindowEvent::PointerScrolled {
                position: _,
                delta_x: _,
                delta_y: _,
            } => defmt::write!(fmt, "PointerScrolled"),
            WindowEvent::PointerExited => defmt::write!(fmt, "PointerExited"),
            WindowEvent::KeyPressed { text: _ } => defmt::write!(fmt, "KeyPressed"),
            WindowEvent::KeyPressRepeated { text: _ } => defmt::write!(fmt, "KeyPressRepeated"),
            WindowEvent::KeyReleased { text: _ } => defmt::write!(fmt, "KeyReleased"),
            WindowEvent::ScaleFactorChanged { scale_factor: _ } => {
                defmt::write!(fmt, "ScaleFactorChanged")
            }
            WindowEvent::Resized { size: _ } => defmt::write!(fmt, "Resized"),
            WindowEvent::CloseRequested => defmt::write!(fmt, "CloseRequested"),
            WindowEvent::WindowActiveChanged(_) => defmt::write!(fmt, "WindowActiveChanged"),
            _ => defmt::write!(fmt, "Unknown window event"),
        }
    }
}
