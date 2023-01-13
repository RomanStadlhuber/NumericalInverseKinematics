function robot = getRigidBodyTree(dhparams)
    rbt = rigidBodyTree;

    for i = 1:size(dhparams,1)
        body = rigidBody(string("body"+i));
        joint = rigidBodyJoint(string("R"+i), "revolute");
        setFixedTransform(joint, dhparams(1,:), "dh");
        body.Joint = joint;

        if(i == 1)
            addBody(rbt, body, "base");
        else
            addBody(rbt, body, string("body"+(i-1)));
        end
    end

    rbt.DataFormat = "column";

    robot = rbt;
end