#
# Just finds the relevant include directory and librariees used to develop
# with CLAPACK

FIND_PATH(CLAPACK_INCLUDE_DIR clapack.h
   /usr/local/include
   /usr/include
)

FIND_LIBRARY(CLAPACK_LIBRARY
  NAMES
   clapack
  PATHS
   /usr/local/lib
   /usr/lib
)

FIND_LIBRARY(CLAPACK_BLAS_LIBRARY
  NAMES
   blas
  PATHS
   /usr/local/lib
   /usr/lib
)

FIND_LIBRARY(CLAPACK_F2C_LIBRARY
  NAMES
   libf2c
  PATHS
   /usr/local/lib
   /usr/lib
)


SET(CLAPACK_FOUND 0)
IF(CLAPACK_INCLUDE_DIR)
  IF(CLAPACK_LIBRARY)
    SET(CLAPACK_FOUND 1)
  ENDIF(CLAPACK_LIBRARY)
ENDIF(CLAPACK_INCLUDE_DIR)

IF(CLAPACK_FOUND)
  INCLUDE_DIRECTORIES(${CLAPACK_INCLUDE_DIR})
ELSE(CLAPACK_FOUND)
  MESSAGE("PROBLEM: CLAPACK not found.")
ENDIF(CLAPACK_FOUND)

SET(CLAPACK_LIBRARIES ${CLAPACK_LIBRARY} ${CLAPACK_BLAS_LIBRARY} ${CLAPACK_F2C_LIBRARY} )

MARK_AS_ADVANCED(CLAPACK_INCLUDE_DIR
  CLAPACK_LIBRARY
  CLAPACK_BLAS_LIBRARY
  CLAPACK_F2C_LIBRARY)
 