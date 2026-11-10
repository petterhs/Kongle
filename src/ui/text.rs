//! Word wrapping without splitting a UTF-8 character or allocating a paragraph.
pub fn next_line(text: &str, columns: usize) -> (&str, &str) {
    let text = text.trim_start();
    if text.is_empty() || columns == 0 {
        return ("", "");
    }
    let limit = text
        .char_indices()
        .nth(columns)
        .map(|(i, _)| i)
        .unwrap_or(text.len());
    if limit == text.len() {
        return (text, "");
    }
    let end = text[..limit]
        .rfind(char::is_whitespace)
        .filter(|&i| i > 0)
        .unwrap_or(limit);
    (&text[..end], &text[end..])
}
