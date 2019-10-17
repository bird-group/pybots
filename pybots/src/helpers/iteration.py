import itertools

def iter_previous_and_next(some_iterable, fill=True):
    """Iterate with next and previous values
    from: http://stackoverflow.com/a/1012089

    Arguments
        some_iterable: thing to iterate
        fill: optional, fill prev and next so that they return None when they
            do not exist. Defaults True, if false then the iterable will start
            at the second value and terminate at the second to last

    Returns:
        prev_iter_next: iterable with three returns
    """
    prevs, items, nexts = itertools.tee(some_iterable, 3)
    if fill:
        prevs = itertools.chain([None], prevs)
        nexts = itertools.chain(
            itertools.islice(nexts, 1, None), [None])
    else:
        prevs = itertools.islice(prevs, 0, None)
        nexts = itertools.islice(nexts, 2, None)
        items = itertools.islice(items, 1, None)
    return itertools.izip(prevs, items, nexts)

def iter_next(some_iterable, fill=True):
    """Iterate with next value
    from: http://stackoverflow.com/a/1012089

    Arguments
        some_iterable: thing to iterate
        fill: optional, fill next so that it returns None when it does not
            exist. Defaults True, if false then the iterable will terminate at
            the second to last (so that next is still defined)

    Returns:
        iter_next: iterable with three returns

    """
    items, nexts = itertools.tee(some_iterable, 2)
    if fill:
        nexts = itertools.chain(
            itertools.islice(nexts, 1, None), [None])
    else:
        nexts = itertools.islice(nexts, 1, None)
        items = itertools.islice(items, 0, None)
    return itertools.izip(items, nexts)

