#pragma once

class Skill;
class DribbleSkill;
class KeepAwaySkill;
class KickPassSkill;
class ChipPassSkill;
class ShootSkill;

class SkillVisitor
{
   public:
    virtual void visit(const Skill &skill) = delete;

    virtual void visit(const KeepAwaySkill &skill) = 0;
    virtual void visit(const KickPassSkill &skill) = 0;
    virtual void visit(const ChipPassSkill &skill) = 0;
    virtual void visit(const ShootSkill &skill)    = 0;
};
