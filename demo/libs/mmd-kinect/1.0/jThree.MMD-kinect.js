/**
 * jThree.MMD-kinect.js JavaScript Library v1.0.0
 * http://www.jthree.com/
 * Created by m2wasabi on 2014/12/10.
 *
 * requires jQuery.js
 * requires jThree.js 2.0.0
 * requires jThree.MMD.js 1.4.1
 *
 * The MIT License
 *
 * Copyright (c) 2014 Matsuda Mitsuhide, katwat and other contributors
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Date: 2014-12-10
 */

jThree.MMD.kinect = {
    init: function(mmd, kinectBoneList, startJoints) {
        this.mmd = mmd.three(0);
        var bones = this.bones = this.mmd.children[0].bones;
        this.kinectBoneList = kinectBoneList;
        this.startJoints = startJoints;
        // kinectBoneList在初始化的时候，只设置了每个数组元素的name, child属性
        // 这为它再设置index属性，表示这个Kinect关节点对应的mmd模型中的关节点
        // 的索引，后续要通过这个索引去获取bones[index]结构体
        $.each(this.kinectBoneList, function(key, val) {
            bones.forEach(function(bone, idx) {
                if (bone.name === val.name) {
                    val.index = idx;
                    return false;
                }
            });
        });
    },
    pose: function(kinectData) {
        var vec = new THREE.Vector3();
        var vec2 = new THREE.Vector3();
        var data = kinectData;
        var mmd = this.mmd;
        var bones = this.bones;
        var list = this.kinectBoneList;

        // 这一坨代码主要就是调整骨骼的“中心点”，控制整个模型
        //
        // SpinBase是Kinect中骨骼中心点
        // bones[0].name="センター"，中文就是“中心点”
        //
        // 把Kinect中心的坐标，作为mmd中的模型坐标，先转换到世界坐标
        // 再转换到bones[0].parent的局部坐标中
        //
        // 最终只影响到bones[0].position，即模型在世界坐标中的位置，并
        // 没有旋转
        {
            // 对模型进行缩放，值越大，看起来越小
            vec.copy(data.SpineBase).multiplyScalar(5);
            // 左手系 <-> 右手系変換
            vec.z = -vec.z;
            mmd.localToWorld(vec);
            bones[0].parent.updateMatrixWorld();
            bones[0].parent.worldToLocal(vec);
            bones[0].position.copy(vec);
        }

        // 这一坨代码其实是没有用的，注释掉也能正常运行
        //
        // localToWorld只是将vec从mmd的模型坐标转换成世界坐标，并没有
        // 其它地方使用这个vec变量
        {
            // 立方体の位置をキネクトにあわせる
            // 将立方体的位置与Kinect对齐
            var n = 0;
            $.each(data, function(key, val) {
                // 对模型进行缩放，值越大，看起来越小
                vec.copy(this).multiplyScalar(5);
                // 左手系 <-> 右手系変換
                vec.z = -vec.z;
                mmd.localToWorld(vec);
            });
        }

        // 遍历5个关节段
        this.startJoints.forEach(function(key, idx) {
            var bone;
            var child;

            // 这一行可以控制只处理某一个关节段，比如右上肢，其它部位不变
            //if (!(idx == 1))
            //    return;

            while (list[key].child) {
                bone = bones[list[key].index];                   // mmd骨骼信息
                bone_child = bones[list[list[key].child].index]; // mmd骨骼子节点信息
                kbone_pos = data[key];                           // Kinect节点坐标
                kbone_child_pos = data[list[key].child];         // Kinect子节点坐标

                // Kinect子节点坐标，左右手坐标系转换，转换到模型坐标上
                vec.copy(kbone_child_pos);
                vec.z = -vec.z;
                bone.parent.worldToLocal(vec);

                // Kinect当前坐标，左右手坐标系转换，转换到模型坐标上
                vec2.copy(kbone_pos);
                vec2.z = -vec2.z;
                bone.parent.worldToLocal(vec2);

                // 子节点-当前节点的坐标，得到两个向量的差
                vec.subVectors(vec, vec2);

                // 定义法向量
                var vecVertical = new THREE.Vector3();

                // 定义骨骼矢量以进行替换, .position是相对于父节点坐标系的相对坐标
                var boneVector = new THREE.Vector3();
                boneVector.copy(bone_child.position);

                // 规范化每个向量
                boneVector.normalize();
                vec.normalize();

                // 求骨骼的四元数。
                // 参照：http://lolengine.net/blog/2014/02/24/quaternion-from-two-vectors-final
                // 通过两个向量的内积计算两个向量的夹角
                var r = boneVector.dot(vec) + 1;
                if (r < 0.000001) {
                    // 如果内积在误差范围0内，则假定法线垂直于z轴或x轴
                    r = 0;
                    if (Math.abs(boneVector.x) > Math.abs(boneVector.z)) {
                        v1.set(-boneVector.y, boneVector.x, 0);
                    } else {
                        v1.set(0, -boneVector.z, boneVector.y);
                    }
                } else {
                    // 计算根据Kinect差异和原始骨骼法线计算的骨骼目标向量
                    vecVertical.crossVectors(boneVector, vec);
                }

                // 将法线向量和旋转角度指定给四元数
                bone.quaternion.set(vecVertical.x, vecVertical.y, vecVertical.z, r);

                // 规范化四元数
                bone.quaternion.normalize();

                // 在世界坐标中反映骨骼矩阵
                bone.parent.updateMatrixWorld(true);

                // 继续处理子节点
                key = list[key].child;
            }
        });
    }
};
