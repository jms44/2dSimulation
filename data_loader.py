import numpy as np
import glob
import json


class DataLoader:

    xIndexFormats = [np.arange(0,23), np.arange(0,23)]
    yIndexFormats = [np.arange(35,40), np.arange(30,35)]

    @staticmethod
    def output_to_dict(x, labelFormat = 0):
        dict = {}
        if labelFormat == 0:
            dict['targetSpeeds'] = x
        return dict

    @staticmethod
    def get_observation(arm, door, labelFormat = 0):

        #info for input
        currentPose = arm.get_points()
        currentAngles = arm.get_angles()
        doorPose = door.getPose()
        knobPose = door.getKnobPose()
        knobClawDif = [knobPose[0] - currentPose[-1][0], knobPose[1] - currentPose[-1][1]]
        motorSpeeds = arm.get_target_speeds()

        #info for output
        #targetPose = self.arm.get_target_pose()
        #targetAngles = self.arm.get_target_angles()
        #targetSpeeds = self.arm.get_target_speeds()

        values = []
        values.extend(DataLoader.flattenList(currentPose))
        values.extend(DataLoader.flattenList(currentAngles))
        values.extend(DataLoader.flattenList(doorPose))
        values.extend(DataLoader.flattenList(knobPose))
        values.extend(DataLoader.flattenList(knobClawDif))
        values.extend(DataLoader.flattenList(motorSpeeds))
        print(len(motorSpeeds))
        return np.array(values)


    def __init__(self, batchSize = 12, labelFormat = 0, data_dir = "./data/runs/"):
        self.indexLookup = None
        self.data = self.getCumulativeNPArray(data_dir)
        self.batchSize = batchSize
        self.labelFormat = labelFormat
        self.reset()

    def __iter__(self):
        self.reset()
        return self

    def __next__(self):
        batchData = self.getNextBatch()
        if batchData is None:
            raise StopIteration
        self.batchCount += 1
        return batchData

    @staticmethod
    def flattenList(list):
        flat_list = []
        for sublist in list:
            try:
                sublist = iter(sublist)
            except Exception:
                sublist = [sublist,]
            for item in sublist:
                flat_list.append(item)
        return flat_list

    def getCumulativeNPArray(self, dir_path):
        run_files = glob.glob(dir_path + '*')
        runList = []
        for file in run_files:
            data = json.load(open(file, 'r'))
            run = data['runs']
            for point in run:
                annot = []
                update = self.indexLookup is None
                if update: self.indexLookup = [0]
                annot.extend(self.flattenList(point['currentPose']))#annot[0] = point['currentPose']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['currentAngles']))#annot[1] = point['currentAngles']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['doorPose']))#annot[2] = point['doorPose']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['knobPose']))#annot[3] = point['knobPose']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['knobClawDif']))#annot[4] = point['knobClawDif']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['motorSpeeds']))#annot[5] = point['motorSpeeds']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['targetPose']))#annot[6] = point['targetPose']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['targetAngles']))#annot[7] = point['targetAngles']
                if update: self.indexLookup.append(len(annot))
                annot.extend(self.flattenList(point['targetSpeeds']))#annot[8] = point['targetSpeeds']
                if update: self.indexLookup.append(len(annot))
                runList.append(np.array(annot))


        npArray = np.array(runList)
        np.random.shuffle(npArray)
        return npArray

    def reset(self):
        self.batchCount = 0

    def getNextBatch(self):
        startIndex = self.batchCount * self.batchSize
        if startIndex >= len(self.data):
            return None
        endIndex = startIndex + min(len(self.data) - startIndex, self.batchSize)
        batchData = self.data[startIndex:endIndex]
        return batchData[:, self.xIndexFormats[self.labelFormat]], batchData[:, self.yIndexFormats[self.labelFormat]]

if __name__ == "__main__":
    loader = DataLoader(2)
    sum = 0
    print(loader.indexLookup)
    for x, y in iter(loader):
        sum += len(y)
