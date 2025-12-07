
import os

def fix_sidebar_labels(docs_dir):
    fixed_files_count = 0
    for root, _, files in os.walk(docs_dir):
        for file in files:
            if file.endswith(".md"):
                filepath = os.path.join(root, file)
                with open(filepath, 'r', encoding='utf-8') as f:
                    content = f.readlines()

                new_content = []
                changed = False
                for line in content:
                    stripped_line = line.strip()
                    if stripped_line.startswith("sidebar_label:"):
                        # Extract the value part after "sidebar_label:"
                        value_part_raw = stripped_line[len("sidebar_label:"):
].strip()

                        # Check if it contains a colon AND is not already quoted
                        if ":" in value_part_raw and not (
                            (value_part_raw.startswith("'" ) and value_part_raw.endswith("'")) or
                            (value_part_raw.startswith("\"") and value_part_raw.endswith("\""))
                        ):
                            new_value = "'" + value_part_raw + "'"
                            new_line = f"sidebar_label: {new_value}\n" # Reconstruct the line
                            new_content.append(new_line)
                            changed = True
                            print(f"Fixed: {filepath} - {line.strip()} -> {new_line.strip()}")
                        else:
                            new_content.append(line)
                    else:
                        new_content.append(line)
                
                if changed:
                    fixed_files_count += 1
                    with open(filepath, 'w', encoding='utf-8') as f:
                        f.writelines(new_content)
    return fixed_files_count

if __name__ == "__main__":
    docs_directory = "digital-book/docs"
    fixed_count = fix_sidebar_labels(docs_directory)
    if fixed_count > 0:
        print(f"Successfully fixed {fixed_count} file(s) with sidebar_label parsing errors.")
    else:
        print("No sidebar_label parsing errors found or fixed.")
