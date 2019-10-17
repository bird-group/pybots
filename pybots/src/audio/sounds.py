import os

def beep(duration, frequency):
    """ Play a beep sound

    This requires installation of the sox package

    Arguments:
        duration: time in seconds to beep for
        frequency: tone frequency

    Returns:
        no returns
    """
    os.system('play --no-show-progress "|sox -n -p synth ' + str(duration) +
        ' sin ' + str(frequency) + '"')
