from typing import List, Union
import numpy as np

class FileWriter:
    def __init__(self, file_path: str):
        self._file_path = file_path
        self._file = open(file_path, "w")

    def __del__(self):
        self.close()

    def close(self):
        self._file.close()

    def write_line(self, data: Union[List, np.ndarray], split_char: str = " "):
        for i in range(len(data)):
            self._file.write(str(data[i]))
            if i != len(data) - 1:
                self._file.write(split_char)
        self._file.write("\n")

        