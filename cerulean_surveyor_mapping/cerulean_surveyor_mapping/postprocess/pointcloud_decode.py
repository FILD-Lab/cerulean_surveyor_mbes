import numpy as np

# sensor_msgs/msg/PointField datatype constants
_DATATYPE_TO_NUMPY = {
    1: np.int8,
    2: np.uint8,
    3: np.int16,
    4: np.uint16,
    5: np.int32,
    6: np.uint32,
    7: np.float32,
    8: np.float64,
}


def decode_pointcloud2(msg, field_names):
    """Decode a PointCloud2-like message into a dict of 1D float64 numpy
    arrays keyed by field_names. Works on either a real sensor_msgs.msg
    PointCloud2 or a rosbags-deserialized equivalent, since it only touches
    plain attributes (data, fields, point_step, width, height, is_bigendian)
    rather than depending on either message class family. Assumes count=1
    per field.
    """
    endian = '>' if msg.is_bigendian else '<'
    field_by_name = {f.name: f for f in msg.fields}

    names, formats, offsets = [], [], []
    for name in field_names:
        field = field_by_name[name]
        np_type = _DATATYPE_TO_NUMPY[field.datatype]
        names.append(name)
        formats.append(endian + np.dtype(np_type).char)
        offsets.append(field.offset)

    dtype = np.dtype({'names': names, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})

    num_points = msg.width * msg.height
    raw = np.frombuffer(bytes(msg.data), dtype=dtype, count=num_points)

    return {name: raw[name].astype(np.float64, copy=True) for name in field_names}
