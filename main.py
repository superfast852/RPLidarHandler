import matplotlib.pyplot as plt
from LidarHandler import RP_A1, RPLidarException, np


class AnimatedWindow:
    def __init__(self):
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.gca()
        self.figid = id(self.fig)
    def scatter(self, points):
        self.ax.scatter(*zip(*points))
    def refresh(self):
        if id(plt.gcf()) != self.figid:
            raise ValueError("Window does not exist.")
        plt.draw()
        plt.pause(0.0001)
    def clear(self):
        self.ax.cla()


def run():
    use_quality = True
    lidar = RP_A1(scan_type="express", threaded=False, quality=use_quality)
    window = AnimatedWindow()
    max_side = 13000
    ancles = np.array([[-max_side, -max_side], [-max_side, max_side], [max_side, max_side], [max_side, -max_side], [-max_side, -max_side]]).T

    try:
        while True:
            # basic frame

            window.clear()
            window.ax.plot(*ancles)

            # data processing
            points = -lidar.readCartesian()

            if lidar.get_quality:
                window.ax.scatter(*zip(*points[:2]), c=points[2] if use_quality else None)
            window.scatter(points)
            window.scatter(((0, 0), ))  # see the center
            window.refresh()

    except RPLidarException:
        print("Something has gone wrong!")
    except ValueError as e:
        if str(e) == "Window does not exist.":
            pass
        else:
            print(e)
    except KeyboardInterrupt:
        pass
    finally:
        lidar.exit()


if __name__ == '__main__':
    run()