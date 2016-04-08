#!/usr/bin/env python
#*******************************************************************************
#*   Ledger Blue
#*   (c) 2016 Ledger
#*
#*  Licensed under the Apache License, Version 2.0 (the "License");
#*  you may not use this file except in compliance with the License.
#*  You may obtain a copy of the License at
#*
#*      http://www.apache.org/licenses/LICENSE-2.0
#*
#*  Unless required by applicable law or agreed to in writing, software
#*  distributed under the License is distributed on an "AS IS" BASIS,
#*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#*  See the License for the specific language governing permissions and
#*  limitations under the License.
#********************************************************************************
from ledgerblue.comm import getDongle
from secp256k1 import PublicKey

textToSign = ""
while True:
	data = raw_input("Enter text to sign, end with an empty line : ")
	if len(data) == 0:
		break
	textToSign += data + "\n"

dongle = getDongle(True)
publicKey = dongle.exchange(bytes("8004000000".decode('hex')))
print "publicKey " + str(publicKey).encode('hex')
apdu = bytes("80028000".decode('hex')) + chr(len(textToSign)) + bytes(textToSign)
signature = dongle.exchange(apdu)
print "signature " + str(signature).encode('hex')
publicKey = PublicKey(bytes(publicKey), raw=True)
signature = publicKey.ecdsa_deserialize(bytes(signature))
print "verified " + str(publicKey.ecdsa_verify(bytes(textToSign), signature))

