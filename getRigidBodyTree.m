function robot = getRigidBodyTree(dhParams, jointTypes)
    rbt = rigidBodyTree;
    if exist('jointTypes', 'var')
        if(size(jointTypes,2) == size(dhParams,1))
            jTypes = jointTypes;
        else
            throw(MException('myComponent:inputError','Dimension of JointTypes does not match number of Joints in DH-Parameters'))
        end
    else
        throw(MException('myComponent:noSuchVariable','JointTypes not found'))
    end


    for i = 1:size(dhParams,1)
        switch jTypes(i)
            case 'fixed'
                jointAbr = string("F"+i);
            case 'revolute'
                jointAbr = string("R"+i);
            case 'prismatic'
                jointAbr = string("P"+i);
            otherwise
                throw(MException('','No Joint type matching %s', jTypes(i)))
        end

        body = rigidBody(string("body"+i));
        joint = rigidBodyJoint(jointAbr, jTypes(i));
        setFixedTransform(joint, dhParams(i,:), "dh");
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