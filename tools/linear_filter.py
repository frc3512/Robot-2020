import math

from circular_buffer import CircularBuffer


class LinearFilter:

    def __init__(self, ff_gains: list[float], fb_gains: list[float]):
        """Create a linear FIR or IIR filter.

        Keyword arguments:
        ff_gains -- The "feed forward" or FIR gains.
        fb_gains -- The "feed back" or IIR gains.
        """
        self.__inputs = CircularBuffer(len(ff_gains))
        self.__outputs = CircularBuffer(len(fb_gains))
        self.__input_gains = ff_gains
        self.__output_gains = fb_gains

    # Static methods to create commonly used filters
    @staticmethod
    def single_pole_iir(time_constant: float, period: float):
        """Creates a one-pole IIR low-pass filter of the form:

        y[n] = (1 - gain) * x[n] + gain * y[n-1]

        where gain = e<sup>-dt / T</sup>, T is the time constant in seconds

        This filter is stable for time constants greater than zero.

        Keyword arguments:
        time_constant -- The discrete-time time constant in seconds.
        period -- The period in seconds between samples taken by the user.
        """
        gain = math.exp(-period / time_constant)
        return LinearFilter(1.0 - gain, -gain)

    @staticmethod
    def high_pass(time_constant: float, period: float):
        """Creates a first-order high-pass filter of the form:

        y[n] = gain * x[n] + (-gain) * x[n-1] + gain * y[n-1]

        where gain = e<sup>-dt / T</sup>, T is the time constant in seconds

        This filter is stable for time constants greater than zero.

        Keyword arguments:
        time_constant -- The discrete-time time constant in seconds.
        period -- The period in seconds between samples taken by the user.
        """
        gain = math.exp(-period / time_constant)
        return LinearFilter([gain, -gain], [-gain])

    @staticmethod
    def moving_average(taps: int):
        """Creates a K-tap FIR moving average filter of the form:

        y[n] = 1/k * (x[k] + x[k-1] + … + x[0])

        This filter is always stable.

        Keyword arguments:
        taps -- The number of samples to average over. Higher = smoother but
                slower.
        """
        assert taps > 0

        gains = [1.0 / taps for i in range(taps)]
        return LinearFilter(gains, [])

    def reset(self):
        """Reset the filter state.
        """
        self.__inputs.reset()
        self.__outputs.reset()

    def calculate(self, input: float) -> float:
        """Calculates the next value of the filter.

        Keyword arguments:
        input -- Current input value.

        Returns:
        The filtered value at this step.
        """
        ret_val = 0.0

        # Rotate the inputs
        self.__inputs.push_front(input)

        # Calculate the new value
        for i in range(len(self.__input_gains)):
            ret_val += self.__inputs[i] * self.__input_gains[i]
        for i in range(len(self.__output_gains)):
            ret_val -= self.__outputs[i] * self.__output_gains[i]

        # Rotate the outputs
        self.__outputs.push_front(ret_val)

        return ret_val
