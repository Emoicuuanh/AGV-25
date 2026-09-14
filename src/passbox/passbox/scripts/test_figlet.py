
import pyfiglet

def generate_header(text):
    ascii_art = pyfiglet.figlet_format(text, font="banner")
    # Replace spaces with dots, but keep newlines
    lines = ascii_art.split('\n')
    formatted_lines = []
    # Remove empty first and last lines if present, though banner usually has them
    # The user example has explicit start and end lines with dots
    
    # Just replace spaces with dots to see if it matches
    for line in lines:
        if not line: continue
        formatted_line = line.replace(' ', '.')
        formatted_lines.append(formatted_line)
    
    return "\n".join(formatted_lines)

print("--- REQUEST_ENTER_LIFT ---")
print(generate_header("GOING_TO_TEMP_POSE"))

