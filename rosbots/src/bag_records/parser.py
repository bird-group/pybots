import pdb

import os
import copy

import rosbag

import numpy
import scipy.io

import bag_records.records
import bag_records.rosgraph_records

class BagParser(object):
    """an object to parse bags, you should make an inherited class to use this
    """
    def __init__(self, bag_path=None, topics=None, additional_records={}):
        """Constructor

        All arguments are optional, if no bag is specified then the parser will
        wait for an explicit parse command with a bag. If no topics are
        specified then it will parse all topics it knows about.

        Arguments:
            additional_records: records which should be added to the base class
                This class starts with none, this should be extended by
                inherited classes for their uses. For instance each workspace
                overlay can inherit from the one above and extend it...also can
                be used to temporarily add records for debugging

        Returns:
            no returns
        """
        self._records = {}
        self._records.update(additional_records)
        self._active_records = []
        self._bag_path = bag_path

        self._records = {
            '/rosout': bag_records.rosgraph_records.Log(),
            }

        if additional_records:
            self._records.update(additional_records)
        
        if bag_path is not None:
            self.parse_bag(bag_path, topics)
        return

    def parse_bag(self, bag_path, topics=None, close=True):
        """Parse a bag

        Arguments:
            bag_path: path to the bag to parse or a list of bags to parse
            topics: topics to parse, if not specified all topics will be read

        Returns:
            no returns
        """
        if isinstance(bag_path, list):
            for bag in bag_path:
                self.parse_bag(bag, topics=topics, close=False)

            for r in self._records.values():
                r.close()
            self._active_records.sort()
            return

        assert os.path.exists(bag_path), 'file {} not found'.format(bag_path)

        self._bag_path = bag_path

        if topics is None:
            topics = self._records.keys()

        bag = rosbag.Bag(bag_path)

        for topic, msg, bag_stamp in bag.read_messages(topics=topics):
            if topic in self._records:
                try:
                    self._records[topic].add_message(msg, bag_stamp)
                except AssertionError as error:
                    raise AssertionError(
                        'error in parsing topic {}\n\t'.format(topic) +
                        error.message)
                if topic not in self._active_records:
                    self._active_records.append(topic)

        bag.close()

        if close:
            for r in self._records.values():
                r.close()
            self._active_records.sort()

    def get_record(self, topic, field=None):
        """Get a record from the bag

        Arguments:
            topic: the topic to retrieve
            field: the field within a topic to retrieve. Optional, if not
                specified then the topic record object will be returned.

        Returns:
            record: the requested record. A record object corresponding to the
                type of the topic requested. If a field was requested, then it
                will be a record object corresponding to the type of that field.
        """
        assert topic in self._records, 'topic, {} not found'.format(topic)

        record = self._records[topic]

        if not field:
            return record

        assert record.has_field[field], 'field, {} not in topic'.format(field)

        sub_record = record.get_field(field)
        return sub_record

    @property
    def records(self):
        """List all records available in this parser that contain data

        Arguments:
            no arguments

        Returns:
            records: list of all records which contain data
        """
        return self._active_records

    def __getitem__(self, key):
        """Allow this to be indexed.

        If key is a string then this will look up a record, just as get_record.

        Arguments:
            key: string to get a record

        Returns:
            val: the field or entry requested
        """
        assert isinstance(key, basestring), 'must specify valid string entry'

        return self.get_record(key)

    def to_mat(self, file_name=None):
        """Save all records in this parser as a matlab file

        Arguments:
            file_name: the name of the file you'd like to use. If not specified
                then it will default to the same name as the bag

        Returns:
            no returns
        """
        if self._bag_path is None:
            return

        output_records = {}
        for key in self._active_records:
            new_key = key.replace('/', '_')[1:]
            output_records[new_key] = self._records[key].to_dict

        if not file_name:
            file_name = '{}.mat'.format(self._bag_path)

        scipy.io.savemat(
            file_name,
            output_records,
            long_field_names=True,
            appendmat=True)

    def slice_by_time(self, start_time, end_time, time_reference='msg'):
        """Slice the bag and return a segment limited by indices

        Arguments:
            start_time: timestamp of the beginning of the slice
            end_time: timestamp of the end of the slice
            time_reference: slice according to time referenced to message time
                or bag time. Defaults to message time. Will return None if
                message time not available set 'msg' for message time and 'bag'
                for bag time

        Returns:
            slice: an object of same type and structure but with data reduced to
                only those indices between the limits
        """
        new_record = copy.deepcopy(self)
        for topic, record in new_record._records.iteritems():
            new_record._records[topic] = record.slice_by_time(
                start_time, end_time, time_reference)
        return new_record
