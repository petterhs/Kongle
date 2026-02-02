use embedded_graphics::{
    geometry::Size,
    image::ImageRaw,
    mono_font::{mapping::ASCII, DecorationDimensions, MonoFont},
};

/// 44x85 pixel 54 point size extra bold monospace font.
pub const JETBRAINS_FONT_54_POINT_EXTRA_BOLD: MonoFont = MonoFont {
    image: ImageRaw::new(
        include_bytes!("../../fonts/jetbrains_font_54_extra_bold.raw"),
        704,
    ),
    glyph_mapping: &ASCII,
    character_size: Size::new(44, 85),
    character_spacing: 2,
    baseline: 71,
    underline: DecorationDimensions::new(71 + 2, 1),
    strikethrough: DecorationDimensions::new(85 / 2, 1),
};
