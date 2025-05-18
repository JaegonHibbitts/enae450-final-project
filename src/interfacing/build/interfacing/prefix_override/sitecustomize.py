import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/icf3ver/ENAE450/final_project/src/interfacing/install/interfacing'
