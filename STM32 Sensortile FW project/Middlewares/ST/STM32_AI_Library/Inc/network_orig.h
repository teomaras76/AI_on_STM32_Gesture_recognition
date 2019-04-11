#if   defined(NN_GMP)
        #include "..\HAR_GMP\Inc\network.h"
#elif defined(NN_IGN)
        #include "..\HAR_IGN\Inc\network.h"
#elif defined(NN_IGN_WSDM)
        #include "..\HAR_IGN_WSDM\Inc\network.h"
#elif defined(NN_ASC)
        #include "..\ASC\Inc\network.h"
#else
        #error "Define according to the used NN network NN_GMP, NN_IGN , NN_IGN_WSDM  or NN_ASC"
#endif
