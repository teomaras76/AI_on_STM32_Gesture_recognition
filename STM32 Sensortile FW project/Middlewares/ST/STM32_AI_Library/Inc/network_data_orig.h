#if   defined(NN_GMP)
        #include "..\HAR_GMP\Inc\network_data.h"
#elif defined(NN_IGN)
        #include "..\HAR_IGN\Inc\network_data.h"
#elif defined(NN_IGN_WSDM)
        #include "..\HAR_IGN_WSDM\Inc\network_data.h"
#elif defined(NN_ASC)
        #include "..\ASC\Inc\network_data.h"
#else
        #error "Define according to the used NN network NN_GMP, NN_IGN, NN_IGN_WSDM or NN_ASC"
#endif
