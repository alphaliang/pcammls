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

def test_numpy_2_c_buffer():
    ''' convert numpy array to c array buffer '''
    print('-'*10)
    print('test_numpy_2_c_buffer')
    #2d array  test
    data = [list(range(10))]*2
    arr = np.array(data,np.uint16)
    carr = sdk.uint16_t_ARRAY.from_nparray(arr)
    test_array_equal(arr,carr,20)
    #3d numpy array 
    res=[]
    data = [[[1,2]]*5]*2
    arr = np.array(data,np.uint8)
    carr = sdk.uint8_t_ARRAY.from_nparray(arr)
    test_array_equal(arr,carr,20)
    print('test done')


def test_c_buffer_2_numpy():
    ''' convert  c array buffer to numpy array  '''
    print('-'*10)
    print('test_c_buffer_2_numpy')
    carr = sdk.uint8_t_ARRAY(20)
    for x in range(20):
        carr[x]=x
    nparr = carr.as_nparray2d(2,10)
    test_array_equal(nparr,carr,20)
    print('test done')

def test_vect_3f():
    print('-'*10)
    print('test_vect_3f')
    data = []
    for k in range(10):
        data.append([k,k+1,k+2])
    arr = np.array(data,np.float64)
    carr = sdk.TY_VECT_3F_ARRAY.from_nparray(arr)
    arr_out = carr.as_nparray(10)
    for k in range(10):
        for s in range(3):
            if (arr[k][s]!=arr_out[k][s])  : print('{} not equal'.format(k))
    for k in range(10):
        if (arr[k][0]!=carr[k].x)  : print('{} not equal'.format(k))
        if (arr[k][1]!=carr[k].y)  : print('{} not equal'.format(k))
        if (arr[k][2]!=carr[k].z)  : print('{} not equal'.format(k))
    print('test done')
    

def main():
    test_numpy_2_c_buffer()
    test_c_buffer_2_numpy()
    test_vect_3f()

main()



