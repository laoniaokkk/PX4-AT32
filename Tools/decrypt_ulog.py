#!/usr/bin/env python3

from Crypto.PublicKey import RSA
from Crypto.Cipher import PKCS1_OAEP
from Crypto.Cipher import ChaCha20
from Crypto.Hash import SHA256
import binascii
import argparse
#from pathlib import Path
import sys

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="""CLI tool to decrypt an ulog file\n""")
    parser.add_argument("ulog_file", help=".ulog file", nargs='?', default=None)
    parser.add_argument("ulog_key", help=".ulogk, encrypted key", nargs='?', default=None)
    parser.add_argument("rsa_key", help=".pem format key for decrypting the ulog key", nargs='?', default=None)

    args = parser.parse_args()

    # Only generate a key pair, don't sign
    if not args.ulog_file or not args.ulog_key or not args.rsa_key:
        print('Need all arguments, the encrypted ulog file, the key and the key decryption key')
        sys.exit(1);

    # Read the private RSA key to decrypt the cahcha key
    with open(args.rsa_key, 'rb') as f:
        r = RSA.importKey(f.read(), passphrase='')

    # Read the encrypted xchacha key and the nonce
    with open(args.ulog_key, 'rb') as f:
        ulog_key_header = f.read(22)

        # Parse the header
        try:
            # magic
            if not ulog_key_header.startswith(bytearray("ULogKey".encode())):
                raise Exception()
            # version
            if ulog_key_header[7] != 1:
                raise Exception()
            # expected key exchange algorithm (RSA_OAEP)
            if ulog_key_header[16] != 4:
                raise Exception()
            key_size = ulog_key_header[19] << 8 | ulog_key_header[18];
            nonce_size = ulog_key_header[21] << 8 | ulog_key_header[20];
            ulog_key_cipher = f.read(key_size)
            nonce = f.read(nonce_size)
        except:
            print("Keyfile format error")
            sys.exit(1);

    # Decrypt the xchacha key
    cipher_rsa = PKCS1_OAEP.new(r,SHA256)
    ulog_key = cipher_rsa.decrypt(ulog_key_cipher)
    #print(binascii.hexlify(ulog_key))

    # Read and decrypt the .ulgc
    cipher = ChaCha20.new(key=ulog_key, nonce=nonce)
    with open(args.ulog_file, 'rb') as f:
        with open(args.ulog_file.rstrip(args.ulog_file[-1]), 'wb') as out:
            out.write(cipher.decrypt(f.read()))
