# COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models

### [Paper](https://arxiv.org/abs/2409.15146) | [Video](https://youtu.be/dV1J-VXdEJA)

> COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models <br />
> Author: Kehui Liu, Zixin Tang, Dong Wang, Zhigang Wang, Bin Zhao, Xuelong Li
>
<!-- <p align="center">
  <a href="">
    <img src="./media/xx.gif" alt="Logo" width="80%">
  </a>
</p> -->
![Figure](media/COHERENT.jpg)
Leveraging the powerful reasoning capabilities of large language models (LLMs), recent LLM-based robot task planning methods yield promising results. However, they mainly focus on single or multiple homogeneous robots on simple tasks. Practically, complex long-horizon tasks always require collaborations among multiple heterogeneous robots especially with more complex action spaces, which makes these tasks more challenging. To this end, we propose COHERENT, a novel LLM-based task planning framework for collaboration of heterogeneous multi-robot systems including quadrotors, robotic dogs, and robotic arms. Specifically, a Proposal-Execution-Feedback-Adjustment (PEFA) mechanism is designed to decompose and assign actions for individual robots, where a centralized task assigner makes a task planning proposal to decompose the complex task into subtasks, and then assigns subtasks to robot executors. Each robot executor selects a feasible action to implement the assigned subtask and reports self-reflection feedback to the task assigner for plan adjustment. The PEFA loops until the task is completed. Moreover, we create a challenging heterogeneous multi-robot task planning benchmark encompassing 100 complex long-horizon tasks. The experimental results show that our work surpasses the previous methods by a large margin in terms of success rate and execution efficiency.

## Update
The code and benchmark are coming soon.

## Acknowledgement

We adapt codes from some awesome repositories, including [llm-mcts](https://github.com/1989Ryan/llm-mcts), [Co-LLM-Agents](https://github.com/1989Ryan/llm-mcts), [OmniGibson](https://github.com/StanfordVL/OmniGibson)
## Citation

If you use this work or find it helpful, please consider citing: (bibtex)
```
@misc{liu2024coherentcollaborationheterogeneousmultirobot,
      title={COHERENT: Collaboration of Heterogeneous Multi-Robot System with Large Language Models}, 
      author={Kehui Liu and Zixin Tang and Dong Wang and Zhigang Wang and Bin Zhao and Xuelong Li},
      year={2024},
      eprint={2409.15146},
      archivePrefix={arXiv},
      primaryClass={cs.RO},
      url={https://arxiv.org/abs/2409.15146}, 
}
```
