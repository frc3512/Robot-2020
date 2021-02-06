class CircularBuffer:
    """This is a simple circular buffer so we don't need to "bucket brigade"
    copy old values.
    """

    def __init__(self, size: int):
        self.__data = [0.0] * size

        # Index of element at front of buffer
        self.__front = 0

        # Number of elements used in buffer
        self.__length = 0

    def size(self) -> int:
        """Returns number of elements in buffer
        """
        return self.__length

    def front(self) -> float:
        """Returns value at front of buffer
        """
        return self.__getitem__(0)

    def back(self) -> float:
        """Returns value at back of buffer
        """
        return self.data[(self.__front + self.__length - 1) % len(self.__data)]

    def push_front(self, value: float):
        """Push new value onto front of the buffer. The value at the back is
        overwritten if the buffer is full.
        """
        if len(self.__data) == 0:
            return

        self.__front = self.__modulo_dec(self.__front)

        self.__data[self.__front] = value

        if self.__length < len(self.__data):
            self.__length += 1

    def push_back(self, value: float):
        """Push new value onto back of the buffer. The value at the front is
        overwrittten if the buffer is full.
        """
        if len(self.__data) == 0:
            return

        self.__data[(self.__front + self.__length) % len(self.__data)] = value

        if self.__length < len(self.__data):
            self.__length += 1
        else:
            # Increment front if buffer is full to maintain size
            self.__front = self.__modulo_inc(self.__front)

    def pop_front(self) -> float:
        """Pop value at front of buffer.
        """
        temp = self.__data[self.__front]
        self.__front = self.__modulo_inc(self.__front)
        self.__length -= 1
        return temp

    def pop_back(self) -> float:
        """Pop value at back of buffer.
        """
        self.__length -= 1
        return self.__data[(self.__front + self.__length) % len(self.__data)]

    def reset(self):
        """Sets internal buffer contents to zero.
        """
        self.__data = [0.0 for i in range(len(self.__data))]
        self.__front = 0
        self.__length = 0

    def __modulo_inc(self, index: int) -> int:
        """Increment an index modulo the length of the buffer.

        Returns the result of the modulo operation.
        """
        return (index + 1) % len(self.__data)

    def __modulo_dec(self, index: int) -> int:
        """Decrement an index modulo the length of the buffer.

        Returns the result of the modulo operation.
        """
        if index == 0:
            return len(self.__data) - 1
        else:
            return index - 1

    def __setitem__(self, index: int, data: float):
        self.__data[(self.__front + index) % len(self.__data)] = data

    def __getitem__(self, index: int) -> float:
        return self.__data[(self.__front + index) % len(self.__data)]
