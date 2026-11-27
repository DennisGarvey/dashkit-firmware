import unittest

from zmq_bridge import FRAME, FrameReorder, REORDER_NS, TerminalStatus, UsbStream


class FrameReorderTest(unittest.TestCase):
    def test_each_frame_waits_and_equal_timestamps_are_forwarded(self):
        reorder = FrameReorder()
        first = (1_000_000, 0, 0x101, b'a')
        tied = (1_000_000, 1, 0x102, b'b')
        newer = (41_000_000, 1, 0x103, b'c')
        reorder.push(first, 0)
        reorder.push(tied, 1)
        reorder.push(newer, 1)

        self.assertEqual(list(reorder.ready(REORDER_NS - 1)), [])
        self.assertEqual(list(reorder.ready(REORDER_NS)), [first])
        self.assertEqual(list(reorder.ready(REORDER_NS + 1)), [(1_000_001, *tied[1:]), newer])
        self.assertEqual(reorder.late_dropped, 0)

    def test_late_frame_is_dropped_without_creating_a_usb_sequence_gap(self):
        stream = UsbStream()
        reorder = FrameReorder()

        def receive(seq, timestamp, arrival):
            reorder.push(stream.accept((FRAME, 0, 1, seq, timestamp, (0x101, b'a'))), arrival)
            return list(reorder.ready(arrival + REORDER_NS))

        self.assertEqual(receive(1, 10_000, 0), [(10_000_000, 0, 0x101, b'a')])
        self.assertEqual(receive(2, 9_995, 2 * REORDER_NS), [])
        self.assertEqual(receive(3, 10_001, 4 * REORDER_NS), [(10_001_000, 0, 0x101, b'a')])
        self.assertEqual(reorder.late_dropped, 1)
        self.assertEqual(stream.gaps, [0, 0])
        status = TerminalStatus(stream, reorder, enabled=False)
        self.assertIn('Late Frames Dropped: 1', status._metric_groups()[-1])


if __name__ == '__main__':
    unittest.main()
