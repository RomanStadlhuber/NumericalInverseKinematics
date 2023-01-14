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
                jointAbr = 'F';
            case 'revolute'
                jointAbr = 'R';
            case 'prismatic'
                jointAbr = 'P';
            otherwise
                throw(MException('','No Joint type matching %s', jTypes(i)))
        end

        body = rigidBody(string("body"+i));
        joint = rigidBodyJoint(string(jointAbr+i), jTypes(i));
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