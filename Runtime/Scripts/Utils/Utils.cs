namespace Preliy.Flange
{
    public static class Utils
    {

        public static float[] CopyArray(this float[] value)
        {
            var array = new float[value.Length];
            value.CopyTo(array,0);
            return array;
        }
    }
}
