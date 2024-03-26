from yacs.config import CfgNode as CN

_C = CN()

_C.DATASET = CN()
_C.DATASET.TRAIN = CN()
_C.DATASET.TRAIN.SOURCE_NAME = None
_C.DATASET.TRAIN.SOURCE_PATH = None
_C.DATASET.TRAIN.TARGET_NAME = None
_C.DATASET.TRAIN.TARGET_PATH = None
_C.DATASET.TEST = CN()
_C.DATASET.TEST.SOURCE_NAME = None
_C.DATASET.TEST.SOURCE_PATH = None
_C.DATASET.TEST.TARGET_NAME = None
_C.DATASET.TEST.TARGET_PATH = None

_C.MODEL = CN()
_C.MODEL.NAME = None
_C.MODEL.CHECKPOINT = None

_C.HYPER = CN()
_C.HYPER.EPOCHS = None
_C.HYPER.BATCH_SIZE = None
_C.HYPER.LEARNING_RATE = None

_C.LOSS = CN()
_C.LOSS.EE = False
_C.LOSS.VEC = False
_C.LOSS.COL = False
_C.LOSS.COL_THRESHOLD = None
_C.LOSS.LIM = False
_C.LOSS.ORI = False
_C.LOSS.FIN = False
_C.LOSS.REG = False

_C.INFERENCE = CN()
_C.INFERENCE.MOTION = CN()
_C.INFERENCE.MOTION.SOURCE = None
_C.INFERENCE.MOTION.KEY = None
_C.INFERENCE.H5 = CN()
_C.INFERENCE.H5.BOOL = None
_C.INFERENCE.H5.PATH = None

_C.OTHERS = CN()
_C.OTHERS.SAVE = None
_C.OTHERS.LOG = None
_C.OTHERS.SUMMARY = None
_C.OTHERS.LOG_INTERVAL = None

cfg = _C
