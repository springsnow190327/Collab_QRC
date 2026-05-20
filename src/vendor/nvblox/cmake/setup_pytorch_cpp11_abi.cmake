# This file sets up the PRE_CXX11_ABI_LINKABLE option based on PyTorch's ABI.
# This option avoids any implementations using std::string in their signature in
# header files. Useful for Nvblox PyTorch wrapper compatibility.

if(BUILD_PYTORCH_WRAPPER)
  # Detect PyTorch ABI if not explicitly set by user
  if(NOT DEFINED PRE_CXX11_ABI_LINKABLE)
    execute_process(
      COMMAND python3 -c
              "import torch; print(int(torch._C._GLIBCXX_USE_CXX11_ABI))"
      OUTPUT_VARIABLE PYTORCH_CXX11_ABI
      OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET
      RESULT_VARIABLE PYTORCH_DETECT_RESULT)

    if(PYTORCH_DETECT_RESULT EQUAL 0)
      # PyTorch found - match its ABI
      if(PYTORCH_CXX11_ABI EQUAL 0)
        set(PRE_CXX11_ABI_DEFAULT ON)
        message(STATUS "Detected PyTorch with pre-CXX11 ABI")
      else()
        set(PRE_CXX11_ABI_DEFAULT OFF)
        message(STATUS "Detected PyTorch with CXX11 ABI")
      endif()
    else()
      # PyTorch not found - default to CXX11 ABI
      set(PRE_CXX11_ABI_DEFAULT OFF)
      message(STATUS "PyTorch not detected, defaulting to CXX11 ABI")
    endif()
  endif()

  option(PRE_CXX11_ABI_LINKABLE "Better support pre-C++11 ABI library users"
         ${PRE_CXX11_ABI_DEFAULT})

  if(PRE_CXX11_ABI_LINKABLE)
    message(STATUS "Building with pre-C++11 ABI support")
  else()
    message(STATUS "Building with C++11 ABI")
  endif()
else()
  option(PRE_CXX11_ABI_LINKABLE "Better support pre-C++11 ABI library users"
         OFF)
  message(
    STATUS "Building without pre-C++11 ABI support (PyTorch wrapper disabled)")
endif()
