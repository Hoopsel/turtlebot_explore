class ScanDetect():
    def __init__(self, ranges, angle_min, angle_max, angle_increment):
        self.ranges = [x for x in ranges if x > 0]
        self.angles = [angle_min + i * angle_increment for i, j in enumerate(ranges) if j > 0]
        self.angle_min, self.angle_max, self.angle_increment = angle_min, angle_max, angle_increment

    def segmentise(self):
        d_thd = 0.05
        segments = []
        segment = []

        for i in range(len(self.ranges) - 1):
            segment.append((self.ranges[i], self.angles[i]))
            if segment is [] or abs(self.ranges[i+1] - self.ranges[i]) < d_thd:
                pass
            else:
                segments.append(segment)
                segment= []

        if segment is not []: segments.append(segment)

        print len(segments)
        print [(x[0], x[-1]) for x in segments]
        
        return segments



