import numpy as np
import hdf5storage
from pymavlink import mavutil

filename = 'solar.bin'
mlog = mavutil.mavlink_connection(filename)

data = {}
mlog.rewind()
while True:
    msg = mlog.recv_match(blocking=False)
    if msg is None:
        break
    msg_type = msg.get_type()
    if msg_type == 'UNKNOWN':
        continue
    d = msg.to_dict()
    if msg_type not in data:
        data[msg_type] = []
    data[msg_type].append(d)

# Convert lists of dicts to dict of arrays for MATLAB
mat_data = {}
for msg_type, records in data.items():
    if not records:
        continue
    fields = {}
    for k in records[0].keys():
        if k == 'mavpackettype':
            continue
        values = [r[k] for r in records]
        # Try numeric first; fall back to string cell array
        try:
            arr = np.array(values, dtype=np.float64)
        except (ValueError, TypeError):
            # Store strings as object array so MATLAB sees a cell array
            arr = np.array([str(v) for v in values], dtype=object)
        fields[k] = arr
    if fields:
        mat_data[msg_type] = fields

# Write MATLAB v7.3 (HDF5) format — handles all field types and large arrays
hdf5storage.savemat('solar.mat', mat_data, format='7.3', matlab_compatible=True)

print(f'Saved {len(mat_data)} message types to solar.mat (v7.3 / HDF5):')
for msg_type, fields in sorted(mat_data.items()):
    n = len(next(iter(fields.values())))
    field_names = ', '.join(fields.keys())
    print(f'  {msg_type:12s}  {n:8d} samples  [{field_names}]')
