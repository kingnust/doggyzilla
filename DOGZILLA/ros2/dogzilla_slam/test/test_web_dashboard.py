from html.parser import HTMLParser
from pathlib import Path
import re
import unittest


STATIC_DIRECTORY = (
    Path(__file__).resolve().parents[1]
    / 'dogzilla_slam'
    / 'web_static'
)


class IdCollector(HTMLParser):
    def __init__(self):
        super().__init__()
        self.ids = []
        self.scripts = []

    def handle_starttag(self, tag, attrs):
        attributes = dict(attrs)
        if 'id' in attributes:
            self.ids.append(attributes['id'])
        if tag == 'script':
            self.scripts.append(attributes)


class WebDashboardTest(unittest.TestCase):
    def test_dashboard_assets_exist_and_dom_ids_are_wired(self):
        html = (STATIC_DIRECTORY / 'index.html').read_text()
        javascript = (STATIC_DIRECTORY / 'app.js').read_text()
        stylesheet = (STATIC_DIRECTORY / 'styles.css').read_text()

        parser = IdCollector()
        parser.feed(html)
        self.assertEqual(len(parser.ids), len(set(parser.ids)))
        self.assertGreater(len(stylesheet), 1000)
        self.assertIn(
            {'src': '/assets/app.js', 'defer': None},
            parser.scripts,
        )

        references = set()
        pattern = re.compile(
            r"elements(?:\.([A-Za-z][A-Za-z0-9_-]*)|\[['\"]([^'\"]+)['\"]\])"
        )
        for dotted, indexed in pattern.findall(javascript):
            references.add(dotted or indexed)
        self.assertEqual(references - set(parser.ids), set())

    def test_dashboard_contains_required_operator_surfaces(self):
        html = (STATIC_DIRECTORY / 'index.html').read_text()
        for element_id in (
            'battery-value',
            'pose-x',
            'robot-mode',
            'active-task',
            'delivery-form',
            'task-list',
            'estop',
            'reset-estop',
            'nav-state',
            'joint-count',
            'map-canvas',
            'map-stage',
            'map-message',
            'route-preview',
            'location-name',
            'save-location',
            'location-list',
            'vision-frame',
            'vision-mode',
            'vision-color',
            'vision-apply',
            'vision-result',
            'vision-status',
        ):
            with self.subTest(element_id=element_id):
                self.assertIn(f'id="{element_id}"', html)

    def test_map_editor_uses_clicks_and_fixed_heading_choices(self):
        html = (STATIC_DIRECTORY / 'index.html').read_text()
        javascript = (STATIC_DIRECTORY / 'app.js').read_text()

        self.assertIn("addEventListener('click'", javascript)
        self.assertNotIn("addEventListener('pointermove'", javascript)
        self.assertNotIn('draggable=', html)
        self.assertIn('<select id="pickup-yaw"', html)
        self.assertIn('<select id="dropoff-yaw"', html)
        self.assertIn('North · 90°', html)


if __name__ == '__main__':
    unittest.main()
