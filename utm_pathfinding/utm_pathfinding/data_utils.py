import pickle

def get_pickle_data(pickle_file):
    print('Loading pickle file: ' + pickle_file + '.pkl')
    with open(pickle_file+'.pkl', 'rb') as f:
        data = pickle.load(f)
    return data

#function to save dictionary to pickle
def save_to_pickle(pickle_file_name, data):
    with open(pickle_file_name+'.pkl', 'wb') as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
        

