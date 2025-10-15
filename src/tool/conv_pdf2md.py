import pdfplumber
from operator import itemgetter
import os

def extract_contents(page):
    """PDFページから重複しないテキストおよび表データを抽出する。"""
    contents = []
    tables = page.find_tables(table_settings={"horizontal_strategy": "text"})
    
    # テーブルに含まれないすべてのテキストデータを抽出
    non_table_content = page
    for table in tables:
        non_table_content = non_table_content.outside_bbox(table.bbox)
        
    for line in non_table_content.extract_text_lines():
        contents.append({'top': line['top'], 'text': line['text']})

    # すべての表データを抽出
    for table in tables:
        contents.append({'top': table.bbox[1], 'table': table})

    # 行を上部位置でソート
    contents = sorted(contents, key=itemgetter('top'))

    return contents

def sanitize_cell(cell):
    """セルの内容を整理する。"""
    if cell is None:
        return ""
    else:
        # すべての種類の空白を削除し、文字列であることを保証
        return ' '.join(str(cell).split())
    
def save_to_markdown(directory, file_prefix, page_number, contents):
    """抽出した内容をMarkdownファイルに保存する。"""
    filename = f"{file_prefix}_page_{page_number}.md"
    filepath = os.path.join(directory, filename)
    if not os.path.exists(directory):
        os.makedirs(directory)

    with open(filepath, 'w', encoding='utf-8') as file:
        file.write(f'# Page {page_number}\n\n')

        for content in contents:
            if 'text' in content:
                # テキスト内容を段落として書き込む
                file.write(f"{content['text']}\n\n")
            elif 'table' in content:
                table = content['table']
                unsanitized_table = table.extract()  # 表オブジェクトからデータを抽出
                sanitized_table = [[sanitize_cell(cell) for cell in row] for row in unsanitized_table]
                # ヘッダーセパレーターを作成
                header_separator = '|:--' * len(sanitized_table[0]) + ':|\n'
                # テーブルデータをMarkdown形式に変換
                for i, row in enumerate(sanitized_table):
                    md_row = '| ' + ' | '.join(row) + ' |\n'
                    file.write(md_row)
                    # 最初の行（ヘッダー行）の後にヘッダーセパレーターを追加
                    if i == 0:
                        file.write(header_separator)
                # テーブルの後にセパレーターを追加（任意）
                file.write('\n---\n\n')

        print(f"{page_number} has been written to {filename}")

def process_pdf(pdf_path, output_dir, file_prefix):
    """PDFの各ページを処理して別々のMarkdownファイルに保存する。"""

    with pdfplumber.open(pdf_path) as pdf:
        for i, page in enumerate(pdf.pages):
            lines = extract_contents(page)
            save_to_markdown(output_dir, file_prefix, i+1, lines)

# PDFファイルのパス、出力ディレクトリ、ファイルプレフィックスを定義
pdf_path = 'RS02-CN.pdf'
output_dir = 'output'
file_prefix = 'iryohi_md'

# 処理を実行
process_pdf(pdf_path, output_dir, file_prefix)
