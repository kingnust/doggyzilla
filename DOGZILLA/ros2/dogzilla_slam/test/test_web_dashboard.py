from html.parser import HTMLParser
import math
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
            'vision-safety',
            'patrol-alert-list',
            'patrol-name',
            'patrol-spacing',
            'patrol-repeats',
            'patrol-area-list',
            'patrol-preview-button',
            'patrol-save',
            'patrol-queue',
            'patrol-status',
            'map-zoom-out',
            'map-zoom-in',
            'map-zoom-fit',
            'map-zoom-level',
            'keepout-name',
            'keepout-zone-list',
            'keepout-save',
            'keepout-delete',
            'keepout-status',
            'drive-speed',
            'drive-speed-value',
            'drive-turn',
            'drive-turn-value',
            'drive-state',
            'drive-message',
            'nav-diagnostics',
            'nav-diagnostics-detail',
            'nav-tuning',
            'nav-tuning-detail',
            'nav-tuning-marker',
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
        self.assertIn('Draw patrol area', html)
        self.assertIn("'/api/v1/patrol-areas/preview'", javascript)
        self.assertIn('Draw keepout', html)
        self.assertIn("'/api/v1/keepout-zones'", javascript)
        self.assertIn('const scale = fitScale * mapZoom', javascript)
        self.assertIn('(screenX - mapView.offsetX) / mapView.scale', javascript)
        self.assertIn('(screenY - mapView.offsetY) / mapView.scale', javascript)
        self.assertIn(
            'waypoints[target] = { ...normalizedPoint, yaw: selectedYaw }',
            javascript,
        )
        self.assertIn('pointToPolygonDistance(point, zone.polygon)', javascript)
        self.assertIn('mapSnapshot.keepout_clearance_m', javascript)

    def test_map_click_transform_is_zoom_invariant_for_rotated_maps(self):
        width = 640
        height = 480
        resolution = 0.05
        origin_x = -3.2
        origin_y = 1.7
        origin_yaw = 0.31
        local_x = 143.25
        local_y = 222.75
        cosine = math.cos(origin_yaw)
        sine = math.sin(origin_yaw)
        world_x = origin_x + cosine * local_x * resolution \
            - sine * local_y * resolution
        world_y = origin_y + sine * local_x * resolution \
            + cosine * local_y * resolution

        for zoom in (1.0, 1.5, 2.0, 3.0, 4.0):
            with self.subTest(zoom=zoom):
                fit_scale = min((900 - 36) / width, (700 - 36) / height)
                scale = fit_scale * zoom
                offset_x = (900 - width * scale) / 2
                offset_y = (700 - height * scale) / 2
                screen_x = offset_x + local_x * scale
                screen_y = offset_y + (height - local_y) * scale

                clicked_local_x = (screen_x - offset_x) / scale
                clicked_local_y = height - (screen_y - offset_y) / scale
                clicked_world_x = origin_x \
                    + cosine * clicked_local_x * resolution \
                    - sine * clicked_local_y * resolution
                clicked_world_y = origin_y \
                    + sine * clicked_local_x * resolution \
                    + cosine * clicked_local_y * resolution

                self.assertAlmostEqual(clicked_world_x, world_x, places=12)
                self.assertAlmostEqual(clicked_world_y, world_y, places=12)

    def test_dashboard_exposes_general_and_confirmed_danger_modes(self):
        html = (STATIC_DIRECTORY / 'index.html').read_text()
        javascript = (STATIC_DIRECTORY / 'app.js').read_text()

        self.assertIn('value="raw"', html)
        self.assertIn('value="objects"', html)
        self.assertIn('value="dangerous-objects"', html)
        self.assertIn('value="floor-hazards"', html)
        self.assertIn('value="patrol"', html)
        self.assertIn('id="patrol-alert-list"', html)
        self.assertIn(
            "['objects', 'dangerous-objects', 'floor-hazards', 'patrol']",
            javascript,
        )
        self.assertIn(
            "['hazard.confirmed', 'person.confirmed'].includes(event.type)",
            javascript,
        )

    def test_dashboard_uses_password_and_autonomous_integer_sliders(self):
        html = (STATIC_DIRECTORY / 'index.html').read_text()
        javascript = (STATIC_DIRECTORY / 'app.js').read_text()

        self.assertIn('id="password"', html)
        self.assertNotIn('id="token"', html)
        self.assertIn("headers.set('X-Dogzilla-Password', password)", javascript)
        self.assertIn('<p class="eyebrow">Autonomous navigation</p>', html)
        self.assertIn('id="drive-speed" type="range" min="1" max="9" step="1"', html)
        self.assertIn('id="drive-turn" type="range" min="1" max="9" step="1"', html)
        self.assertIn("'/api/v1/autonomy/speed'", javascript)
        self.assertNotIn('data-drive=', html)
        self.assertNotIn("'/api/v1/drive'", javascript)
        self.assertIn("event.type === 'navigation.warning'", javascript)
        self.assertIn('Warning · monitoring only', javascript)
        self.assertIn("'/api/v1/navigation/tuning/marker'", javascript)
        self.assertIn('Marker only: it does not slow, stop, or cancel', html)


if __name__ == '__main__':
    unittest.main()
