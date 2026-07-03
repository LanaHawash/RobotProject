from robot_project.world.object import WorldObject


class ObjectManager:

    def __init__(self):
        self.objects = []

    def update(self, detections):
        self.objects.clear()

        for index, detection in enumerate(detections):

            obj = WorldObject(
                object_id=index,
                label=detection["class"],
                confidence=detection["confidence"],
                distance_mm=detection["distance_mm"],
                center_x=detection["center"][0],
                center_y=detection["center"][1],
            )

            self.objects.append(obj)

    def get_all(self):
        return self.objects
    

    def get_closest_object(self):

        if not self.objects:
            return None

        return min(self.objects, key=lambda obj: obj.distance_mm)


    def get_objects_by_class(self, label):

        return [
            obj
            for obj in self.objects
            if obj.label == label
        ]


    def get_closest_object_by_class(self, label):

        objects = self.get_objects_by_class(label)

        if not objects:
            return None

        return min(objects, key=lambda obj: obj.distance_mm)


    def has_object(self, label):

        return any(
            obj.label == label
            for obj in self.objects
        )


    def get_object_count(self):

        return len(self.objects)   