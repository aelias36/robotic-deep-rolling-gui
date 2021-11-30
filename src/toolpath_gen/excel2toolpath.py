'''
excel2toolpath.py
Alex Elias

Converts an excel file with toolpath parameters into toolpath files
'''

DOE_EXCEL_FILE = "RPI_DOE_RDR_V2.xlsx"
VERSION_NUMBER = "2"

import yaml
import pandas as pd

from force_toolpath_gen import GenFullToolpath


def main():
    with open('../toolpath_constants.yaml', 'r') as file:
        params = yaml.safe_load(file)


    df = pd.read_excel(DOE_EXCEL_FILE)

    for index, row in df.iterrows():
        if index == 0:
            assert(row['load'] == 'lb')
            assert(row['step over'] == 'in')
            assert(row['corner radius'] == 'in')
            continue

        print(f"\n\n{index}:\n{row}")
        print(row['corner radius'])
        params['margin_length'] = row['corner radius'] * 25.4e-3 # in -> m
        params['stepover'] = row['step over'] * 25.4e-3 # in -> m
        params['f_max'] = row['load'] * 4.448 # lb -> N

        gen = GenFullToolpath(params)

        toolpath = gen.toolpath()
        with open(f"RTX_sample_{int(row['Sample number'])}v{VERSION_NUMBER}.toolpath", 'w') as file:
          file.write(toolpath)

if __name__ == '__main__':
    main()