
import os
import re

def validate_metadata(docs_dir):
    errors = []
    markdown_files = []
    for root, _, files in os.walk(docs_dir):
        for file in files:
            if file.endswith(".md"):
                markdown_files.append(os.path.join(root, file))

    if not markdown_files:
        errors.append(f"No markdown files found in {docs_dir}")
        return False, errors

    for filepath in markdown_files:
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.readlines()

        # Check for unique ID structure for all sub-sections (e.g., ## Physical AI Principles {id: ch1-sec1-principles})
        for i, line in enumerate(content):
            if re.match(r'^\s*##+\s', line):  # Matches any header (##, ###, etc.)
                if not re.search(r'\{id:\s*ch\d+-sec\d+-[a-zA-Z0-9-]+\}', line):
                    errors.append(f"File: {filepath}, Line {i+1}: Header without required ID format: {line.strip()}")
    
    if errors:
        return False, errors
    else:
        return True, ["All markdown files have correct metadata chunk format."]

if __name__ == "__main__":
    docs_directory = "digital-book/docs"
    is_valid, messages = validate_metadata(docs_directory)
    if is_valid:
        print("Validation successful:")
        for msg in messages:
            print(f"- {msg}")
    else:
        print("Validation failed:")
        for msg in messages:
            print(f"- {msg}")
