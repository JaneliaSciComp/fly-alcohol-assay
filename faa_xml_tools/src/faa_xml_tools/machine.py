from __future__ import print_function
import os
import os.path
from xml.etree import ElementTree


def read_machine_file():
    """
    Reads the machine definition xml ROS launch file  FAA_CONFIG/machine/faa.machine

    Returns a list containg a dictionary of attributes for each machine in the
    machine file.
    """
    faa_config_dir = os.environ['FAA_CONFIG']
    faa_machine_file = os.path.join(faa_config_dir, 'machine', 'faa.machine')
    tree = ElementTree.parse(faa_machine_file)
    machine_elem_list = tree.findall('machine')
    machine_list = []
    for machine_elem in machine_elem_list:
        machine_list.append(machine_elem.attrib)
    return machine_list



# -----------------------------------------------------------------------------
if __name__ == '__main__':
    machine_list = read_machine_file()
    print(machine_list)




