namespace Workflow.Core.Tests;

public class CalculatorTests
{
    [Test]
    public void TestAdd()
    {
        Assert.That(Calculator.Add(1, 1), Is.EqualTo(2));
        
        Assert.That(Calculator.Add(1, 2), Is.EqualTo(3));
        Assert.That(Calculator.Add(2, 1), Is.EqualTo(3));
        
        Assert.That(Calculator.Add(11, 32), Is.EqualTo(43));
    }
}