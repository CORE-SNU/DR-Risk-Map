function obs_idx = choose_obs(obs_state, x, t, min_dist)
    obs_idx = [];
    if norm(x(1:2,t)-obs_state(:,t,1)) <= min_dist
        obs_idx = [obs_idx 1];
    end
    if norm(x(1:2,t)-obs_state(:,t,2)) <= min_dist
        obs_idx = [obs_idx 2];
    end
    if norm(x(1:2,t)-obs_state(:,t,3)) <= min_dist
        obs_idx = [obs_idx 3];
    end
end