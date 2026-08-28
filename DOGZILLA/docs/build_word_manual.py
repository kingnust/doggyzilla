#!/usr/bin/env python3
"""Build one Word manual from the maintained DOGZILLA Markdown chapters."""

from __future__ import annotations

from datetime import date
from html import escape
from html.parser import HTMLParser
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import zipfile
import xml.etree.ElementTree as ElementTree

import mistune


DOCUMENT_NAME = 'DOGZILLA_S2_COMPLETE_DEVELOPER_DOCUMENTATION.docx'
CHAPTERS = (
    'README.md',
    'DEVELOPER_HANDOFF.md',
    'FRAMEWORK.md',
    'PIPELINES.md',
    'FIRMWARE_AND_SERIAL.md',
    'ROS_INTERFACES.md',
    'OPERATIONS_RUNBOOK.md',
    'COMPUTER_VISION.md',
    'URDF_RTABMAP_MONO.md',
    'DEVELOPMENT_ROADMAP.md',
)


class _PlainText(HTMLParser):
    def __init__(self):
        super().__init__()
        self.parts: list[str] = []

    def handle_data(self, data):
        self.parts.append(data)

    def value(self):
        return ''.join(self.parts).strip()


def _plain_text(value):
    parser = _PlainText()
    parser.feed(value)
    return parser.value()


def _slug(value):
    result = re.sub(r'[^a-z0-9]+', '-', value.lower()).strip('-')
    return result or 'section'


def _render_chapter(markdown, chapter_slug):
    renderer = mistune.HTMLRenderer(escape=False)
    render = mistune.create_markdown(
        renderer=renderer,
        plugins=['strikethrough', 'table', 'task_lists', 'url'],
    )
    html = render(markdown)
    headings = []
    used = set()

    def add_heading(match):
        level = int(match.group(1))
        content = match.group(2)
        title = _plain_text(content)
        base = f'{chapter_slug}-{_slug(title)}'
        heading_id = base
        suffix = 2
        while heading_id in used:
            heading_id = f'{base}-{suffix}'
            suffix += 1
        used.add(heading_id)
        headings.append((level, title, heading_id))
        return f'<h{level} id="{heading_id}">{content}</h{level}>'

    html = re.sub(
        r'<h([1-6])>(.*?)</h\1>',
        add_heading,
        html,
        flags=re.DOTALL,
    )
    html = html.replace(
        '<table>',
        '<table width="100%" cellspacing="0" cellpadding="4" '
        'style="width:100%; table-layout:fixed">',
    )
    html = html.replace('<th>', '<th style="word-wrap:break-word">')
    html = html.replace('<td>', '<td style="word-wrap:break-word">')
    return html, headings


def _rewrite_chapter_links(html, chapter_anchors):
    def replace(match):
        filename = Path(match.group(1)).name
        anchor = chapter_anchors.get(filename)
        if anchor is None:
            return match.group(0)
        return f'href="#{anchor}"'

    return re.sub(
        r'href="(?:\./)?([^"#]+\.md)(?:#[^"]*)?"',
        replace,
        html,
        flags=re.IGNORECASE,
    )


def _table_of_contents(chapters):
    items = []
    for chapter in chapters:
        visible = [item for item in chapter['headings'] if item[0] <= 2]
        if not visible:
            continue
        title = visible[0][1]
        items.append(
            '<li class="toc-chapter">'
            f'<a href="#{chapter["anchor"]}">{escape(title)}</a>'
        )
        subsections = [item for item in visible[1:] if item[0] == 2]
        if subsections:
            items.append('<ul>')
            for _, heading, heading_id in subsections:
                items.append(
                    f'<li><a href="#{heading_id}">{escape(heading)}</a></li>'
                )
            items.append('</ul>')
        items.append('</li>')
    return '<ol class="toc">' + ''.join(items) + '</ol>'


def _build_html(docs_dir):
    chapter_anchors = {
        name: f'chapter-{_slug(Path(name).stem)}' for name in CHAPTERS
    }
    chapters = []
    for name in CHAPTERS:
        path = docs_dir / name
        if not path.is_file():
            raise FileNotFoundError(f'required chapter is missing: {path}')
        chapter_slug = _slug(path.stem)
        body, headings = _render_chapter(
            path.read_text(encoding='utf-8'),
            chapter_slug,
        )
        chapters.append({
            'name': name,
            'anchor': chapter_anchors[name],
            'body': body,
            'headings': headings,
        })

    for chapter in chapters:
        chapter['body'] = _rewrite_chapter_links(
            chapter['body'], chapter_anchors
        )

    pipeline_image = (docs_dir / 'DOGZILLA_PIPELINE_SLIDE.png').resolve()
    if not pipeline_image.is_file():
        raise FileNotFoundError(f'pipeline image is missing: {pipeline_image}')

    sections = []
    for chapter in chapters:
        extra = ''
        if chapter['name'] == 'PIPELINES.md':
            extra = (
                '<figure>'
                f'<img src="{pipeline_image.name}" '
                'width="624" height="351" '
                'alt="DOGZILLA four-layer system pipeline">'
                '<figcaption>Figure 1. Simplified four-layer DOGZILLA '
                'pipeline for presentations.</figcaption>'
                '</figure>'
            )
        sections.append(
            f'<div class="chapter" id="{chapter["anchor"]}">'
            f'<p class="source" style="page-break-before: always">'
            f'Source chapter: docs/{chapter["name"]}</p>'
            f'{chapter["body"]}{extra}</div>'
        )

    generated = date.today().isoformat()
    contents = _table_of_contents(chapters)
    return f'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>DOGZILLA S2 Complete Developer Documentation</title>
<style>
@page {{ size: A4; margin: 20mm 18mm 20mm 18mm; }}
body {{
  color: #17202a;
  font-family: "Liberation Sans", Arial, sans-serif;
  font-size: 10.5pt;
  line-height: 1.35;
}}
.title-page {{
  page-break-after: always;
  text-align: center;
  padding-top: 55mm;
}}
.title-page h1 {{ color: #102a43; font-size: 30pt; margin-bottom: 8mm; }}
.title-page .subtitle {{ color: #486581; font-size: 16pt; }}
.title-page .meta {{ color: #627d98; margin-top: 28mm; }}
.contents-page {{ page-break-after: always; }}
.contents-page h1 {{ color: #102a43; border-bottom: 2px solid #2f80ed; }}
.chapter {{ page-break-before: always; }}
.chapter:first-of-type {{ page-break-before: auto; }}
h1 {{ color: #102a43; font-size: 23pt; margin-top: 0; }}
h2 {{ color: #1f5f99; font-size: 16pt; border-bottom: 1px solid #bcccdc; }}
h3 {{ color: #334e68; font-size: 13pt; }}
h4, h5, h6 {{ color: #486581; }}
p, li {{ orphans: 3; widows: 3; }}
a {{ color: #1c65a5; text-decoration: none; }}
.source {{
  color: #829ab1;
  font-size: 8.5pt;
  font-style: italic;
  margin-bottom: 8mm;
}}
.toc {{ padding-left: 8mm; }}
.toc li {{ margin-bottom: 1.5mm; }}
.toc-chapter {{ font-weight: bold; margin-top: 3mm; }}
.toc-chapter ul {{ font-weight: normal; margin-top: 1.5mm; }}
table {{
  border-collapse: collapse;
  width: 100%;
  margin: 4mm 0;
  page-break-inside: avoid;
}}
th {{ background: #d9eaf7; color: #102a43; }}
th, td {{ border: 1px solid #9fb3c8; padding: 2mm; vertical-align: top; }}
pre {{
  background: #f0f4f8;
  border: 1px solid #bcccdc;
  padding: 3mm;
  font-family: "Liberation Mono", monospace;
  font-size: 8.5pt;
  white-space: pre-wrap;
}}
code {{
  background: #f0f4f8;
  color: #7b341e;
  font-family: "Liberation Mono", monospace;
  font-size: 9pt;
}}
pre code {{ color: #17202a; }}
blockquote {{
  border-left: 3px solid #2f80ed;
  color: #486581;
  margin-left: 0;
  padding-left: 4mm;
}}
figure {{ page-break-inside: avoid; text-align: center; margin: 8mm 0; }}
figure img {{ width: 165mm; max-width: 100%; }}
figcaption {{ color: #627d98; font-size: 9pt; font-style: italic; }}
hr {{ border: 0; border-top: 1px solid #bcccdc; }}
</style>
</head>
<body>
<div class="title-page">
  <h1>DOGZILLA S2</h1>
  <p class="subtitle">Complete Developer Documentation</p>
  <p>Yahboom DOGZILLA S2 · Raspberry Pi · ROS 2 Humble</p>
  <p class="meta">Generated {generated}<br>
  Repository: /home/pi/DOGZILLA<br>
  Embedded controller firmware source is not included; verified public
  interfaces and explicit unknowns are documented.</p>
</div>
<div class="contents-page">
  <h1 style="page-break-before: always">Contents</h1>
  {contents}
</div>
{''.join(sections)}
</body>
</html>
'''


def _validate_docx(path):
    if not path.is_file() or path.stat().st_size < 50_000:
        raise RuntimeError('generated Word file is missing or unexpectedly small')
    with zipfile.ZipFile(path) as archive:
        names = set(archive.namelist())
        required = {'[Content_Types].xml', 'word/document.xml'}
        if not required.issubset(names):
            raise RuntimeError('generated file is not a valid Word document')
        if not any(name.startswith('word/media/') for name in names):
            raise RuntimeError('pipeline image was not embedded in the Word file')


def _embed_linked_pipeline_image(path, image_path):
    """Replace LibreOffice's external HTML image link with DOCX media."""
    with zipfile.ZipFile(path) as archive:
        content = {
            item.filename: archive.read(item.filename)
            for item in archive.infolist()
        }
    if any(name.startswith('word/media/') for name in content):
        return

    relationships_name = 'word/_rels/document.xml.rels'
    relationships = ElementTree.fromstring(content[relationships_name])
    relationship = None
    for candidate in relationships:
        target = candidate.attrib.get('Target', '')
        relation_type = candidate.attrib.get('Type', '')
        if (
            relation_type.endswith('/image')
            and Path(target).name == image_path.name
        ):
            relationship = candidate
            break
    if relationship is None:
        raise RuntimeError(
            'LibreOffice did not preserve the pipeline image relationship'
        )

    relationship_id = relationship.attrib['Id']
    relationship.set('Target', f'media/{image_path.name}')
    relationship.attrib.pop('TargetMode', None)
    relationship_namespace = relationships.tag.partition('}')[0].lstrip('{')
    ElementTree.register_namespace('', relationship_namespace)
    content[relationships_name] = ElementTree.tostring(
        relationships,
        encoding='utf-8',
        xml_declaration=True,
    )

    document_name = 'word/document.xml'
    document = content[document_name]
    linked_attribute = f'r:link="{relationship_id}"'.encode()
    embedded_attribute = f'r:embed="{relationship_id}"'.encode()
    if linked_attribute not in document:
        raise RuntimeError('pipeline image link is missing from Word XML')
    content[document_name] = document.replace(
        linked_attribute,
        embedded_attribute,
    )
    content[f'word/media/{image_path.name}'] = image_path.read_bytes()

    types_name = '[Content_Types].xml'
    content_types = ElementTree.fromstring(content[types_name])
    has_png = any(
        item.attrib.get('Extension', '').lower() == 'png'
        for item in content_types
    )
    if not has_png:
        namespace = content_types.tag.partition('}')[0].lstrip('{')
        ElementTree.SubElement(
            content_types,
            f'{{{namespace}}}Default',
            {'Extension': 'png', 'ContentType': 'image/png'},
        )
        ElementTree.register_namespace('', namespace)
        content[types_name] = ElementTree.tostring(
            content_types,
            encoding='utf-8',
            xml_declaration=True,
        )

    replacement = path.with_name(f'{path.stem}.embedded{path.suffix}')
    with zipfile.ZipFile(
        replacement,
        mode='w',
        compression=zipfile.ZIP_DEFLATED,
    ) as archive:
        for name, data in content.items():
            archive.writestr(name, data)
    replacement.replace(path)


def main():
    docs_dir = Path(__file__).resolve().parent
    output = docs_dir / DOCUMENT_NAME
    office = shutil.which('libreoffice')
    if office is None:
        raise RuntimeError('LibreOffice is required to create the Word file')

    html = _build_html(docs_dir)
    with tempfile.TemporaryDirectory(prefix='dogzilla-word-') as temporary:
        temporary_path = Path(temporary)
        source = temporary_path / output.with_suffix('.html').name
        source.write_text(html, encoding='utf-8')
        shutil.copy2(
            docs_dir / 'DOGZILLA_PIPELINE_SLIDE.png',
            temporary_path / 'DOGZILLA_PIPELINE_SLIDE.png',
        )
        profile = temporary_path / 'libreoffice-profile'
        command = [
            office,
            f'-env:UserInstallation={profile.resolve().as_uri()}',
            '--headless',
            '--convert-to',
            'docx:Office Open XML Text',
            '--outdir',
            str(temporary_path),
            str(source),
        ]
        result = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        generated = temporary_path / output.name
        if result.returncode != 0 or not generated.is_file():
            detail = result.stdout.strip() or 'no LibreOffice output'
            raise RuntimeError(f'LibreOffice conversion failed: {detail}')
        _embed_linked_pipeline_image(
            generated,
            temporary_path / 'DOGZILLA_PIPELINE_SLIDE.png',
        )
        _validate_docx(generated)
        shutil.copy2(generated, output)

    _validate_docx(output)
    size_mib = output.stat().st_size / (1024 * 1024)
    print(f'Created {output} ({size_mib:.2f} MiB)')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        raise SystemExit(1)
