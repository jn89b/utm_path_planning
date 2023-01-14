import pickle

def get_pickle_data(pickle_file):
    with open(pickle_file+'.pkl', 'rb') as f:
        map_area = pickle.load(f)
    return map_area
