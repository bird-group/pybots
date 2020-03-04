import pdb

import copy
from six import string_types

import numpy
import scipy.io
import scipy.interpolate

import std_msgs.msg
import geometry_msgs.msg

import matplotlib.pyplot as plt

class RecordBase(object):
    """Base class for record objects
    """
    def __init__(self, has_msg_time=False, interpolate=True):
        """Constructor

        Arguments:
            has_msg_time: indicate if this record has a header for message time.
                Defaults True
            interpolate: indicates if this record can be interpolated (i.e. is
                it atomic or a time series). Defaults True

        Returns:
            class instance
        """
        self._has_msg_time = has_msg_time
        self._interpolate = True

        # the number of entries in this record
        self._n = 0

        # if you have a message with an array of some other message in it then
        # you should specify the type of that record in this dict. That field
        # will then get a list of its entry type for each record. If a field
        # name does not appear in this dict then it is built up as a single
        # entry per record.
        self._array_types = {}
        return

    def add_message(self, msg, bag_time, msg_time=None):
        """Add data from a message to the record

        Arguments:
            msg: the message to add
            bag_time: rospy.Time indicating when the message was added to the
                bag
            msg_time: rospy.Time giving the messages timestamp (optional)

        Returns:
            no returns
        """
        self._n += 1

        if self._has_msg_time:
            if msg_time is not None:
                self._fields['msg_time'].append(msg_time.to_sec())
            else:
                self._fields['msg_time'].append(msg.header.stamp.to_sec())
                msg_time = msg.header.stamp

        self._fields['bag_time'].append(bag_time.to_sec())

        # iterate through the fields this message says it has. For each field
        # make sure we have it in the message, then either add the data for that
        # field to our record, or pass the data to the next record down
        for key, val in self._fields.items():
            if key == 'bag_time' or key == 'msg_time':
                continue
            assert hasattr(msg, key), 'field: {} not found in {}'.format(
                key, type(msg))
            field = getattr(msg, key)
            if isinstance(field, list):
                self.add_array(key, field, bag_time, msg_time)
            elif isinstance(val, list) or isinstance(val, tuple):
                val.append(field)
            elif isinstance(val, RecordBase):
                val.add_message(field, bag_time, msg_time)

    def add_array(self, key, field, bag_time, msg_time=None):
        """Add data from an array message to the record

        Arguments:
            key: the key to index the message within the record structure
            field: the field from the rosbag message
            bag_time: rospy.Time indicating when the message was added to the
                bag
            msg_time: rospy.Time giving the messages timestamp (optional)

        Returns:
            no returns
        """
        if msg_time:
            entry = self._array_types[key](True)
        else:
            entry = self._array_types[key](False)

        for datum in field:
            entry.add_message(datum, bag_time, msg_time)

        self._fields[key].append(entry)
        self._n += 1

    def close(self):
        """Close the bag

        Make numpy arrays out of the time stamps and data

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._fields['bag_time'] = numpy.array(self._fields['bag_time'])
        if self._has_msg_time:
            self._fields['msg_time'] = numpy.array(self._fields['msg_time'])

        for key, val in self._fields.items():
            if key is 'bag_time' or key is 'msg_time':
                continue
            if isinstance(val, RecordBase):
                val.close()
            if isinstance(val, list):
                if key not in self._array_types:
                    self._fields[key] = numpy.array(val)
                else:
                    for entry in val:
                        if isinstance(entry, RecordBase):
                            entry.close()

    def has_field(self, field_name):
        """Check if this record or any of its children has a field

        Arguments:
            field_name: the name of the fiel we want to look for

        Returns:
            has_field: true if the fiel exists in this record or its children
        """
        if field_name in self._fields:
            return True

        has_field = False
        for f in self._fields.values():
            if isinstance(f, RecordBase):
                has_field = has_field or f.has_field(field_name)

        return has_field

    def get_field(self, field_name):
        """Get a field from this record

        Returns None if the field is not found

        Arguments:
            field_name: name of the field to retrieve

        Returns:
            field: the field to get
        """
        if not self.has_field(field_name):
            return None

        return copy.deepcopy(self._fields[field_name])

    def time(self, time_reference='msg', t0=None, t_shift=0.0):
        """Get the time vector

        Arguments:
            time_reference: get time referenced to message time or bag time.
                Defaults to message time. Will return None if message time not
                available set 'msg' for message time and 'bag' for bag time
            t0: shift time so that the first time stamp is t0, if it is left
                unspecified then no shift will be applied
            t_shift: shift time by this amount. if left unspecified then no
                shift will be applied. If both t0 and t_shift are specified then
                t0 will take precedence

        Returns:
            time: time vector
        """
        time = None

        if time_reference.lower() == 'msg' and not self._has_msg_time:
            return time

        if time_reference.lower() == 'msg':
            time = copy.deepcopy(self._fields['msg_time'])
        elif time_reference.lower() == 'bag':
            time = copy.deepcopy(self._fields['bag_time'])

        if time is None:
            return time

        if t0 is not None:
            t_shift = time[0] - t0

        time -= t_shift
        return time

    @property
    def fields(self):
        """List fields in this record

        Arguments:
            no arguments

        Fields:
            list of all fields in this record
        """
        return sorted(self._fields.keys())

    def slice_by_time(self, start_time, end_time, time_reference='msg'):
        """Slice the record and return a time limited segment

        Arguments:
            start_time: timestamp of the beginning of the slice
            end_time: timestamp of the end of the slice
            time_reference: slice according to time referenced to message time
                or bag time. Defaults to message time. Will return None if
                message time not available set 'msg' for message time and 'bag'
                for bag time

        Returns:
            slice: an object of the same type and structure but with a reduced
                time extent
        """
        if time_reference.lower == 'msg' and not self._has_msg_time:
            return None

        if len(self) == 0:
            return None

        if time_reference.lower() == 'msg':
            time = self._fields['msg_time']
        elif time_reference.lower() == 'bag':
            time = self._fields['bag_time']

        if start_time > numpy.amax(time) or end_time < numpy.amin(time):
            return None

        start_idx = numpy.amin(numpy.where(time > start_time))
        end_idx = numpy.amax(numpy.where(time < end_time)) + 1

        return self.slice_by_index(start_idx, end_idx)

    def slice_by_index(self, start_index, end_index):
        """Slice the record and return a segment limited by indices

        Arguments:
            start_index: index at beginning of subrecord
            stop_index: index at end of subrecord

        Returns:
            slice: an object of same type and structure but with data reduced to
                only those indices between the limits
        """
        assert end_index > start_index, 'end index must be greater than start'
        slice = copy.deepcopy(self)
        for key, value in self._fields.items():
            if isinstance(value, RecordBase):
                slice._fields[key] = value.slice_by_index(
                    start_index, end_index)
                slice._fields[key]._n = end_index - start_index
                continue

            if len(value) < end_index:
                slice._fields[key] = copy.deepcopy(value)
                continue

            slice._fields[key] = value[start_index:end_index]

        slice._n = end_index - start_index
        return slice

    def shift_time(self, time_reference='msg', t0=None, t_shift=0.0):
        """Shift the record's time reference

        Arguments:
            time_reference: use time referenced to message time or bag time.
                Defaults to message time. Will return False if message time not
                available set 'msg' for message time and 'bag' for bag time
            t0: shift time so that the first time stamp is t0, if it is left
                unspecified then no shift will be applied
            t_shift: shift time by this amount. if left unspecified then no
                shift will be applied. If both t0 and t_shift are specified then
                t0 will take precedence

        Returns:
            success: returns True if the shift was succesful, False if the shift
                could not be applied because the message time was not available
        """
        if (
            time_reference.lower == 'msg' and not
            self._has_msg_time and
            t0 is not None):
            return False

        if t0 is not None:
            if time_reference.lower() == 'msg':
                t_shift = self._fields['msg_time'][0] - t0
            elif time_reference.lower() == 'bag':
                t_shift = self._fields['bag_time'][0] - t0

        if self._has_msg_time:
            self._fields['msg_time'] -= t_shift
        self._fields['bag_time'] -= t_shift
        for key, val in self._fields.items():
            if key == 'msg_time' or key == 'bag_time':
                continue
            if isinstance(val, RecordBase):
                val.shift_time(time_reference, t0, t_shift)

        return True

    def nearest_to_time(self, epoch, time_reference='msg', causal=True):
        """Return the record nearest to specified time

        Arguments:
            epoch: the time of interest
            time_reference: use time referenced to message time or bag time.
                Defaults to message time. Will return False if message time not
                available set 'msg' for message time and 'bag' for bag time
            causal: indicates whether we should look for the nearest record
                before the indicated time, or just the nearest message. Defaults
                to True (record must be from before epoch)

        Returns:
            nearest_record: record instance nearest to epoch
        """
        if time_reference.lower == 'msg' and not self._has_msg_time:
            return False
        if isinstance(epoch, numpy.ndarray) or isinstance(epoch, list):
            nearest_record = [
                self.nearest_to_time(
                    epoch=t,
                    time_reference=time_reference,
                    causal=causal)
                for t in epoch]
            return nearest_record

        dt = numpy.abs(self.time(time_reference=time_reference) - epoch)
        if causal:
            dt[self.time(time_reference=time_reference) > epoch] = numpy.inf
        idx = numpy.argmin(dt)
        return self.slice_by_index(idx, idx + 1)

    def interpolate(self, times, time_reference='msg'):
        """Interpolate the record to a specified time

        Arguments:
            times: numpy array of time stamps to interpolate to
            time_reference: 'msg' or 'bag' specifying the time reference.
                defaults to 'msg'

        Returns:
            record: fields of this record interpolated to the desired time. This
                will go through nested records until it finds a numpy array. it
                will skipy any strings. The user is required to be intelligent
                about asking it to interpolate integer fields or other fields
                which should be discrete...this will interpolate them as
                floating point numbser.

        Raises:
            ValueError:
                if the times requested fall outside of the valid times
                if time_reference is not 'msg' or 'bag'
        """
        if not self._interpolate:
            raise ValueError(
                "record type {} cannot be interpolated".format(self.type))
        if time_reference.lower() != 'bag' and time_reference.lower() !='msg':
            raise ValueError("time_reference must be 'msg' or 'bag'")

        interpolated_record = copy.deepcopy(self)
        for key, value in self._fields.items():
            if isinstance(value, RecordBase):
                interpolated_record._fields[key] = value.interpolate(
                    times, time_reference)
                continue

            if isinstance(value, numpy.ndarray):
                if time_reference.lower() == 'msg':
                    t = self._fields['msg_time']
                else:
                    t = self._fields['bag_time']
                if numpy.amin(times) < numpy.amin(t):
                    raise ValueError(
                        'minimum of requested time falls outside of valid times'
                        'for this record')
                if numpy.amax(times) > numpy.amax(t):
                    raise ValueError(
                        'maximum of requested time falls outside of valid times'
                        'for this record')

                interpolated_record._fields[key] = scipy.interpolate.interp1d(
                    t, value, axis=0)(times)

        return interpolated_record

    def plot_field(self,
        x_field='msg_time',
        y_field='data',
        ax=None,
        plot_spec='',
        label=None,
        scatter=False):
        """Add a field to a matplotlib figure

        Arguments:
            x_field: name of field to plot on the x axis. if this is left
                unspecified then this defaults to msg_time. Nothing is done if
                this is unspecified and there is no msg_time to this record. A
                numpy.ndarray or list to plot can also be specified
            y_field: name of field to plot on y axis. if this is left
                unspecified and this record contains a field called 'data' then
                data will be plotted. Can also pass an array to plot.
            ax: matplotlib axis object to plot into. If not specified then it
                just plots and lets matplotlib figure it out.
            plot_spec: optional plot specification.
            label: optional plot label
            scatter: optional, scatter vs line plot. defaults to False (line)

        Returns:
            plt: handle to plot
        """
        if isinstance(x_field, string_types):
            if x_field not in self._fields:
                return None
            x_field = self._fields[x_field]
        if isinstance(y_field, string_types):
            if y_field not in self._fields:
                return None
            y_field = self._fields[y_field]

        if not scatter:
            if ax is not None:
                return ax.plot(x_field, y_field, plot_spec, label=label)
            else:
                return plt.plot(x_field, y_field, plot_spec, label=label)

        if ax is not None:
            return ax.scatter(x_field, y_field, plot_spec, label=label)
        else:
            return plt.scatter(x_field, y_field, plot_spec, label=label)

    def get_entry(self, idx):
        """Get a full entry at a specified index

        Arguments:
            idx: the index to lookup

        Returns:
            entry: a dictionary with all of values of this message at the
                specified index
        """
        entry = {}
        for field in self.fields:
            if isinstance(self[field], numpy.ndarray):
                entry[field] = self[field][idx]
            if isinstance(self[field], RecordBase):
                entry[field] = self[field].get_entry(idx)
        return entry

    def get_record_at_time(self, t, time_reference='msg', causal=True):
        """Get the record nearest to a time

        Arguments:
            t: the epoch of interest
            time_reference: source of time for comparison. Can be 'msg' for
                message time or 'bag' for bag time. Defaults to 'msg'
            causal: whether to search causally (return the first message after
                time t) or noncaussaly (the message nearest time t). Defaults
                True for causal search

        Returns:
            record: the record of interest
        """
        if time_reference.lower == 'msg' and not self._has_msg_time:
            return None

        if time_reference.lower() == 'msg':
            time = self._fields['msg_time']
        elif time_reference.lower() == 'bag':
            time = self._fields['bag_time']

        if t < time[0] and causal:
            return None

        dt = time - t
        if causal:
            idx = numpy.where(dt > 0)[0][0]
            return self.slice_by_index(idx, idx + 1)
        else:
            idx = numpy.argmin(numpy.abs(dt))
            return self.slice_by_index(idx, idx + 1)

    @property
    def to_dict(self):
        """Convert this to a dictionary

        Arguments
        """
        record_dict = {}
        for field in self.fields:
            if isinstance(self[field], numpy.ndarray):
                record_dict[field] = self[field]
            if isinstance(self[field], RecordBase):
                record_dict[field] = self[field].to_dict
            if isinstance(self[field], list):
                record_dict[field] = [f.to_dict for f in self[field]]
        return record_dict

    def to_mat(self, file_name):
        """Save this record as a mat file

        Arguments:
            file_name: mat file name to save to

        Returns:
            no returns
        """
        scipy.io.savemat(
            file_name,
            self.to_dict,
            long_field_names=True,
            appendmat=True)

    def __getitem__(self, key):
        """Allow this to be indexed.

        If key is a string then this will look up a field, just as get_field. If
        it is an integer then a dictionary of data representing that index entry
        will be returned.

        Arguments:
            key: either a string to get a field or an index to get a value

        Returns:
            val: the field or entry requested
        """
        if isinstance(key, string_types):
            return self.get_field(key)

        assert isinstance(key, int), 'index must be an integer'

        return self.get_entry(key)

    def __contains__(self, key):
        """Check if a field is in a record

        Arguments:
            key: the key we want to check

        Returns:
            in: bool indicating if the key is in our record
        """
        return self.has_field(key)

    def __len__(self):
        """Get the number of entries in this record
        """
        return self._n
