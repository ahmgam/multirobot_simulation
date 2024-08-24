from encryption import EncryptionModule
import os
import rsa
import sys


def generate_keys():
    '''
    generate new public and private key pair
    '''
    
    #generate new public and private key pair
    pk, sk=rsa.newkeys(2048)
    return pk, sk

def store_keys(public_key_file,private_key_file,pk,sk):
    '''
    store public and private key pair in file
    '''
    
    #store public and private key pair in file
    # Save the public key to a file
    with open(public_key_file, 'wb') as f:
        f.write(pk.save_pkcs1())

    # Save the private key to a file
    with open(private_key_file, 'wb') as f:
        f.write(sk.save_pkcs1())
    return None

def load_keys(pk_file,sk_file):
    '''
    load public and private key pair from file
    '''
    #check if key pairs is available
    if os.path.isfile(pk_file) and os.path.isfile(sk_file):
        #load public and private key pair from file
        with open(pk_file, 'rb') as f:
            pk = rsa.PublicKey.load_pkcs1(f.read())
        with open(sk_file, 'rb') as f:
            sk = rsa.PrivateKey.load_pkcs1(f.read())
        return pk, sk
    else:        
        return None, None
  
#get path and node_id from args
public_key_file = "/robot_ws/src/multirobot_sim/files/pk.pem"
private_key_file =  "/robot_ws/src/multirobot_sim/files/sk.pem"
pk, sk = load_keys(public_key_file, private_key_file)
if pk is None or sk is None:
    pk, sk = generate_keys()
    store_keys(public_key_file, private_key_file, pk, sk)