import pcammls as sdk
import numpy as np

def test_array_equal(nparr, carr,sz):
    print('data is {}'.format(nparr))
    res = []
    arr = np.reshape(nparr,(sz))
    for idx in range(sz):
         res.append(carr[idx])
    for idx in range(sz):
        if res[idx]!=arr[idx]:
            print ('not equal at {}'.format(idx))
    print('test ok')

def test_numpy_2_c_buffer():
    print('-'*10)
    print('test_numpy_2_c_buffer')
    data = [list(range(10))]*2
    arr = np.array(data,np.uint16)
    carr = sdk.uint16_t_ARRAY.from_nparray(arr)
    test_array_equal(arr,carr,20)

    #3d numpy array to c buffer array
    res=[]
    data = [[[1,2]]*5]*2
    arr = np.array(data,np.uint8)
    carr = sdk.uint8_t_ARRAY.from_nparray(arr)
    test_array_equal(arr,carr,20)


def test_c_buffer_2_numpy():
    print('-'*10)
    print('test_c_buffer_2_numpy')
    carr = sdk.uint8_t_ARRAY(20)
    for x in range(20):
        carr[x]=x
    nparr = carr.as_nparray2d(2,10)
    test_array_equal(nparr,carr,20)


def main():
    test_numpy_2_c_buffer()
    test_c_buffer_2_numpy()

main()



