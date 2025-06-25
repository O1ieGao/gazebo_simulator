import base64

import M2Crypto


'''
def pri_encrypt(msg, pri_key):
    rsa_pri = M2Crypto.RSA.load_key(pri_key)
    ctxt_pri = rsa_pri.private_encrypt(msg, M2Crypto.RSA.pkcs1_padding)
    ctxt64_pri = ctxt_pri.encode('base64')
    return ctxt64_pri
'''


def encrypt_pri(msg, pri_key):
    rsaPri = M2Crypto.RSA.load_key(pri_key)
    buffer = []
    while msg:
        input = msg[:117]
        tmp = rsaPri.private_encrypt(input, M2Crypto.RSA.pkcs1_padding)
        buffer.append(tmp)
        msg = msg[117:]
    data = base64.b64encode(''.join(buffer))
    return data


def create_signature(msg, pri_key):
    rsa_pri = M2Crypto.RSA.load_key(pri_key)
    signature = rsa_pri.sign(msg)
    return signature


'''
def pub_decrypt(msg, pub_key, sign):
    bio = M2Crypto.BIO.MemoryBuffer(pub_key)
    rsa_pub = M2Crypto.RSA.load_pub_key_bio(bio)
    ctxt_pri = msg.decode("base64")
    output = rsa_pub.public_decrypt(ctxt_pri, M2Crypto.RSA.pkcs1_padding)
    return output
'''


def decrypt_pub(msg, pub_key):
    rsaPub = M2Crypto.RSA.load_pub_key(pub_key)
    try:
        ctxt_pri = msg.decode("base64")
    except Exception as e:
        ctxt_pri = base64.b64decode(msg)
    buffer = []
    while ctxt_pri:
        input = ctxt_pri[:128]
        ctxt_pri = ctxt_pri[128:]
        tmp = rsaPub.public_decrypt(input, M2Crypto.RSA.pkcs1_padding)
        buffer.append(tmp)
    try:
        return ''.join(buffer)
    except Exception as e:
        return b''.join(buffer).decode()


def verify_signature(msg, pub_key, signature):
    try:
        bio = M2Crypto.BIO.MemoryBuffer(pub_key)
        rsa_pub = M2Crypto.RSA.load_pub_key_bio(bio)
        return rsa_pub.verify(msg, signature)
    except Exception as e:
        msg = msg.encode()
        pub_key = pub_key.encode()
        signature = eval(signature)
        bio = M2Crypto.BIO.MemoryBuffer(pub_key)
        rsa_pub = M2Crypto.RSA.load_pub_key_bio(bio)
        return rsa_pub.verify(msg, signature)
