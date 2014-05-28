# Roshlet that repeatedly saves and matches images and visualizes the results

load('rosh_geometry', globals())
r = Rate(1.0)
i = 0
rostype(topics.save_image, msg.kidnapped_robot.SavePlace)
rostype(topics.match, msg.kidnapped_robot.MatchRequest)
rostype(topics.transform_matches, msg.kidnapped_robot.MatchResult)
        
while ok():
    r.sleep()
    loginfo('Last match result is %s', topics.transform_matches[0])
    i += 1
    topics.save_image(now(), i)
    loginfo('Saving image %s at %s', i, transforms.base_footprint('odom_combined').pose)
    topics.match(now())
    
