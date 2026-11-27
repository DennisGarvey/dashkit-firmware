import unittest

from zmq_bridge import FrameReorder, REORDER_NS


class FrameReorderTest(unittest.TestCase):
    def test_each_frame_waits_and_equal_timestamps_are_preserved(self):
        reorder = FrameReorder(preserve_late_timestamps=True)
        first = (1_000_000, 0, 0x101, b'a')
        tied = (1_000_000, 1, 0x102, b'b')
        newer = (41_000_000, 1, 0x103, b'c')
        reorder.push(first, 0)
        reorder.push(tied, 1)
        reorder.push(newer, 1)

        self.assertEqual(list(reorder.ready(REORDER_NS - 1)), [])
        self.assertEqual(list(reorder.ready(REORDER_NS)), [first])
        self.assertEqual(list(reorder.ready(REORDER_NS + 1)), [tied, newer])
        self.assertEqual(reorder.late, 0)

    def test_late_frame_uses_original_timestamp(self):
        reorder = FrameReorder(preserve_late_timestamps=True)
        newer = (10_000_000, 0, 0x101, b'a')
        late = (9_995_000, 1, 0x102, b'b')
        reorder.push(newer, 0)
        self.assertEqual(list(reorder.ready(REORDER_NS)), [newer])
        reorder.push(late, 2 * REORDER_NS)
        self.assertEqual(list(reorder.ready(3 * REORDER_NS)), [late])
        self.assertEqual(reorder.late, 1)


    def test_default_output_stays_strictly_increasing(self):
        reorder = FrameReorder()
        first = (10_000_000, 0, 0x101, b'a')
        tied = (10_000_000, 1, 0x102, b'b')
        late = (9_995_000, 0, 0x103, b'c')
        reorder.push(first, 0)
        reorder.push(tied, 0)
        self.assertEqual(list(reorder.ready(REORDER_NS)), [first, (10_000_001, *tied[1:])])
        self.assertEqual(reorder.late, 0)
        reorder.push(late, 2 * REORDER_NS)
        self.assertEqual(list(reorder.ready(3 * REORDER_NS)), [(10_000_002, *late[1:])])
        self.assertEqual(reorder.late, 1)


if __name__ == '__main__':
    unittest.main()
