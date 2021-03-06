//
//  SSPathSegment.hpp
//  GlobalPlanner
//
//  Created by Satya Prakash on 28/01/14.
//  Copyright (c) 2014 Satya Prakash. All rights reserved.
//

#ifndef __GlobalPlanner__SSPathSegment__
#define __GlobalPlanner__SSPathSegment__

#include "utils/PathSegment.hpp"
#include <iostream>

namespace navigation {
    
    class SSPathSegment :public PathSegment {
        
    };
    
    typedef std::shared_ptr<SSPathSegment> SSPathSegmentPtr;
    
}
#endif /* defined(__GlobalPlanner__SSPathSegment__) */
