#pragma once

#define TRANSFER_WITH_FLAGS(x, metaFlag) transfer.Transfer (x, #x, metaFlag)
#define TRANSFER_WITH_NAME(x, name) transfer.Transfer (x, name)
#define TRANSFER(x) TRANSFER_WITH_NAME(x, #x)