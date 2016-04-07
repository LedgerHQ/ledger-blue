from ledgerblue.comm import getDongle
dongle = getDongle(True)
dongle.exchange(bytes("800200000401020304".decode('hex')))

