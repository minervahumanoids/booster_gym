import tkinter as tk
from tkinter import filedialog, messagebox
from docx import Document
import re

def extract_data_from_docx(file_name):
    doc = Document(file_name)
    raw_text = []

    for para in doc.paragraphs:
        raw_text.append(para.text)
    
    raw_string = "\n".join(raw_text)
    
    sections = re.split(r'\n(?=\d+:)', raw_string.strip())

    outputs = []
    
    for section in sections:
        output = process_section(section)
        if output:
            outputs.append(output)

    return outputs

def process_section(section):
    try:
        mass = extract_value(section, "质量") / 1000
        cog_x = extract_value(section, "X") / 1000
        cog_y = extract_value(section, "Y") / 1000
        cog_z = extract_value(section, "Z") / 1000
        
        ixx = extract_value(section, "Lxx") / 1e9
        ixy = -extract_value(section, "Lxy") / 1e9
        ixz = -extract_value(section, "Lxz") / 1e9
        iyy = extract_value(section, "Lyy") / 1e9
        iyz = -extract_value(section, "Lyz") / 1e9
        izz = extract_value(section, "Lzz") / 1e9

        output_string = f"""
#########################################################################################
    <inertial>
      <origin
        xyz="{cog_x} {cog_y} {cog_z}"
        rpy="0 0 0" />
      <mass
        value="{mass}" />
      <inertia
        ixx="{ixx}"
        ixy="{ixy}"
        ixz="{ixz}"
        iyy="{iyy}"
        iyz="{iyz}"
        izz="{izz}" />
    </inertial>
"""
        return output_string
    except ValueError as e:
        print(f"处理段落时出错: {e}")
        return None

def extract_value(section, key):
    pattern = re.compile(rf"{key}\s*=\s*([\d\.\-e]+)")
    match = pattern.search(section)
    if match:
        return float(match.group(1))
    else:
        raise ValueError(f"未找到键: {key}")

def upload_file():
    file_path = filedialog.askopenfilename(filetypes=[("Word Files", "*.docx")])
    if file_path:
        try:
            outputs = extract_data_from_docx(file_path)
            result_text.delete(1.0, tk.END)  
            for i, output in enumerate(outputs, start=1):
                result_text.insert(tk.END, f"{i}: {output.strip()}\n")
        except Exception as e:
            messagebox.showerror("错误", f"处理文件时出错: {e}")

# 创建 GUI 窗口
root = tk.Tk()
root.title("urdf格式转换器")

upload_button = tk.Button(root, text="上传 Word 文件", command=upload_file)
upload_button.pack(pady=10)

result_text = tk.Text(root, width=80, height=30)
result_text.pack(pady=10)

root.mainloop()

