#include "utils.h"

#include "lib/fmt.h"
#include "lib/os.h"

String get_next_file(str pattern, error &err) {
    int i = 1;
    
  retry:
    String path = fmt::sprintf(pattern, i);
    
    os::stat(path, err.ignore(os::ErrNOENT));
    if (err.handle(os::ErrNOENT)) {
        return path;
    }
    
    i++;
    goto retry;
}