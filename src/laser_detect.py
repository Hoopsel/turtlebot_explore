class ScanDetect():
    def __init__(self, ranges, angle_min, angle_max, angle_increment):
        self.ranges = ranges
        self.angle_min, self.angle_max, self.angle_increment = angle_min, angle_max, angle_increment

    def segmentise(self):
        d_thd = 0.1
        segments = []
        segment = []

        for i in range(len(self.ranges) - 1):
            if self.ranges[i] == float("nan"):
                continue
            elif segment is [] or self.ranges[i+1] - self.ranges[i] < d_thd:
                segment.append(self.ranges[i])
            else:
                segments.append(segment)
                segment= []

        if segment is not []: segments.append(segment)

        print len(segments)
        print [(x[0], x[-1]) for x in segments]
        
        return segments



