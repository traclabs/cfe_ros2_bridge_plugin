# cFE NAME: CF_TransactionCmd
# ROS topic: cf_transactioncmd
# Command message
std_msgs/Header header
CFEMSGCommandHeader cmd_header
CFTransactionSeqt ts
CFEntityIdt eid
uint8 chan
uint8[] spare
